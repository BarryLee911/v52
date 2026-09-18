module tt_um_sine_area_detector #(
    parameter integer HANDOFF_WAIT_CYCLES = 80000
) (

    input  wire [7:0] ui_in,//8-bit ADC code

    output wire [7:0] uo_out,//Low 8 bits of area or peak

    input  wire [7:0] uio_in,//uio[4:0]: level before latching

    output wire [7:0] uio_out,//uio[7:5]: upper result bits; uio[0]: result type

    output wire [7:0] uio_oe,//uio[0] becomes an output after handoff

    /* Retained for template compatibility; operation ignores ena. */
    input  wire       ena,

    input  wire       clk,//80 MHz

    input  wire       rst_n
);

    wire _unused = &{ena, 1'b0};

    /* Convert the ADC code to a binary sign. */
    wire adc_sign;
    assign adc_sign = (ui_in >= 8'h80);

    /* Levels 0-22: binary division; level 23: 0.5 mHz at 80 MHz. */
    reg [4:0] divider_exponent;
    reg       config_valid;

    always @* begin
        divider_exponent = uio_in[4:0];
        config_valid     = 1'b0;

        if (uio_in[4:0] <= 5'd23)
            config_valid = 1'b1;
    end

    wire square_wave;
    wire overlap_bit;
    assign overlap_bit = adc_sign == square_wave;

    /* Latch the first valid level after reset. */
    reg       config_latched_valid;
    reg [4:0] divider_exponent_latched;

    /*
     * Sample every 2^level clocks; level 23 uses 78,125,000 clocks.
     */
    reg  [26:0] prescale_count;
    wire [26:0] prescale_terminal_extended;
    wire        sample_tick;

    /* Decode once on the configuration edge; preserve the first sample. */
    reg [26:0] prescale_terminal_latched;
    wire [26:0] config_terminal;
    assign config_terminal =
        (divider_exponent == 5'd23) ? 27'd78124999 :
        (27'd1 << divider_exponent) - 27'd1;
    assign prescale_terminal_extended = prescale_terminal_latched;
    assign sample_tick =
        (prescale_count == prescale_terminal_extended);

    /* Capture the same-time sample before processing it one clock later. */
    reg sample_valid;
    reg [7:0] sample_magnitude;
    reg sample_overlap;
    reg [10:0] sample_position;
    reg stats_reset;
    wire process_sample;
    assign process_sample = sample_valid && !stats_reset;

    /* Count ones in the last 2048 overlap samples. */
    reg [2047:0] overlap_history;
    reg [10:0] history_pointer;
    reg [11:0] running_sum;
    reg        window_full;
    /* Independent square wave */
    assign square_wave = history_pointer[10];

    wire        oldest_overlap;
    wire        effective_oldest_overlap;
    wire [11:0] slide_sum_next;

    assign oldest_overlap = overlap_history[2047];
    /* Unwritten history counts as zero. */
    assign effective_oldest_overlap = window_full ? oldest_overlap : 1'b0;
    assign slide_sum_next =
        ( sample_overlap && !effective_oldest_overlap) ? (running_sum + 12'd1) :
        (!sample_overlap &&  effective_oldest_overlap) ? (running_sum - 12'd1) :
                                            running_sum;

    /* Absolute ADC distance from middle. */
    wire [7:0] adc_magnitude;
    assign adc_magnitude = ui_in[7] ? (ui_in - 8'd128) : (8'd128 - ui_in);

    /* Two peak candidates and the latest sample. */
    reg [7:0] peak_first;
    reg [7:0] peak_second;
    reg [7:0] peak_buffer;
    reg [9:0] peak_first_position;
    reg [9:0] peak_second_position;
    reg [9:0] peak_buffer_position;
    reg peak_first_valid;
    reg peak_second_valid;
    reg peak_buffer_valid;

    reg [7:0] peak_first_next;
    reg [7:0] peak_second_next;
    reg [9:0] peak_first_position_next;
    reg [9:0] peak_second_position_next;
    reg peak_first_valid_next;
    reg peak_second_valid_next;
    wire [7:0] peak_buffer_next;
    wire take_sample;
    assign take_sample = config_latched_valid && sample_tick;
    assign peak_buffer_next = sample_magnitude;

    /* Compare original candidates in parallel, before selecting a source. */
    wire first_alive;
    wire second_alive;
    wire buffer_diff_first;
    wire buffer_diff_second;
    wire new_ge_first;
    wire new_ge_second;
    wire new_ge_buffer;
    wire fill_second;
    wire choose_new_first;
    wire choose_new_second;
    wire first_from_first;
    wire first_from_second;
    wire second_from_first;
    wire second_from_second;
    wire second_from_buffer;

    assign first_alive = peak_first_valid &&
        (peak_first_position != sample_position[9:0]);
    assign second_alive = peak_second_valid &&
        (peak_second_position != sample_position[9:0]);
    assign buffer_diff_first = peak_buffer_position != peak_first_position;
    assign buffer_diff_second = peak_buffer_position != peak_second_position;
    assign new_ge_first = (sample_magnitude >= peak_first);
    assign new_ge_second = (sample_magnitude >= peak_second);
    assign new_ge_buffer = (sample_magnitude >= peak_buffer);

    /* Refill only an empty second slot, without duplicating the survivor. */
    assign fill_second = !(first_alive && second_alive) && peak_buffer_valid &&
        ((first_alive && buffer_diff_first) ||
         (!first_alive && (!second_alive || buffer_diff_second)));
    /* Newer equal samples retain the original priority. */
    assign choose_new_first = !(first_alive || second_alive) ||
        (first_alive && new_ge_first) ||
        (!first_alive && second_alive && new_ge_second);
    assign choose_new_second = !choose_new_first &&
        (!((first_alive && second_alive) || fill_second) ||
         (fill_second && new_ge_buffer) ||
         (!fill_second && new_ge_second));

    /* Mutually exclusive selectors keep values and positions paired. */
    assign first_from_first = !choose_new_first && first_alive;
    assign first_from_second = !choose_new_first && !first_alive;
    assign second_from_first = choose_new_first && first_alive;
    assign second_from_second = (choose_new_first && !first_alive) ||
        (!choose_new_first && !choose_new_second && !fill_second);
    assign second_from_buffer = !choose_new_first && !choose_new_second && fill_second;

    always @* begin
        peak_first_next = ({8{choose_new_first}} & peak_buffer_next) |
            ({8{first_from_first}} & peak_first) |
            ({8{first_from_second}} & peak_second);
        peak_first_position_next = ({10{choose_new_first}} & sample_position[9:0]) |
            ({10{first_from_first}} & peak_first_position) |
            ({10{first_from_second}} & peak_second_position);
        peak_first_valid_next = 1'b1;

        peak_second_next = ({8{choose_new_second}} & peak_buffer_next) |
            ({8{second_from_first}} & peak_first) |
            ({8{second_from_second}} & peak_second) |
            ({8{second_from_buffer}} & peak_buffer);
        peak_second_position_next = ({10{choose_new_second}} & sample_position[9:0]) |
            ({10{second_from_first}} & peak_first_position) |
            ({10{second_from_second}} & peak_second_position) |
            ({10{second_from_buffer}} & peak_buffer_position);
        peak_second_valid_next = choose_new_first ? (first_alive || second_alive) :
            ((first_alive && second_alive) || fill_second || choose_new_second);
    end

    /* Wait 1 ms at 80 MHz before driving uio[0]. */
    localparam integer HANDOFF_CYCLES =
        (HANDOFF_WAIT_CYCLES < 1) ? 1 : HANDOFF_WAIT_CYCLES;
    /* Up to 131072 clocks. */
    localparam [16:0] HANDOFF_LAST = HANDOFF_CYCLES - 1;
    reg [16:0] handoff_count;
    reg output_ready;
    /* Register data and its type on the same edge. */
    wire [10:0] area_latest;
    reg [10:0] output_data;
    reg output_kind;
    wire send_peak;
    assign area_latest = running_sum[11] ? 11'd2047 : running_sum[10:0];
    // Keep the output selector known during reset.
    assign send_peak = rst_n && output_ready && !output_kind;
    /* Front end and interface retain the original external reset timing. */
    always @(posedge clk) begin
        stats_reset <= !rst_n;
        if (!rst_n) begin
            config_latched_valid <= 1'b0;
            divider_exponent_latched <= 5'd0;
            prescale_terminal_latched <= 27'd0;
            prescale_count <= 27'd0;
            history_pointer <= 11'd0;
            sample_valid <= 1'b0;
            sample_magnitude <= 8'd0;
            sample_overlap <= 1'b0;
            sample_position <= 11'd0;
            handoff_count <= 0;
            output_ready <= 1'b0;
            output_data <= 11'd0;
            output_kind <= 1'b0;
        end else begin
            if (!config_latched_valid && config_valid) begin
                divider_exponent_latched <= divider_exponent;
                prescale_terminal_latched <= config_terminal;
                config_latched_valid <= 1'b1;
            end
            if (config_latched_valid)
                prescale_count <= sample_tick ? 27'd0 : prescale_count + 27'd1;

            sample_valid <= take_sample;
            if (take_sample) begin
                sample_magnitude <= adc_magnitude;
                sample_overlap <= overlap_bit;
                sample_position <= history_pointer;
                history_pointer <= history_pointer + 11'd1;
            end
            if (config_latched_valid && !output_ready) begin
                if (handoff_count == HANDOFF_LAST)
                    output_ready <= 1'b1;
                else
                    handoff_count <= handoff_count + 1'b1;
            end

            output_kind <= send_peak;
            /* Hide old statistics while the delayed reset clears them. */
            output_data <= stats_reset ? 11'd0 :
                (send_peak ? {3'b000, peak_first} : area_latest);
        end
    end

    /* Backend reset follows the external reset by one clock. */
    always @(posedge clk) begin
        if (stats_reset) begin
            running_sum <= 12'd0;
            window_full <= 1'b0;
            peak_first <= 8'd0;
            peak_second <= 8'd0;
            peak_buffer <= 8'd0;
            peak_first_position <= 10'd0;
            peak_second_position <= 10'd0;
            peak_buffer_position <= 10'd0;
            peak_first_valid <= 1'b0;
            peak_second_valid <= 1'b0;
            peak_buffer_valid <= 1'b0;
        end else if (sample_valid) begin
            overlap_history <= {overlap_history[2046:0], sample_overlap};
            if (sample_position == 11'd2047)
                window_full <= 1'b1;
            running_sum <= slide_sum_next;
            peak_first <= peak_first_next;
            peak_second <= peak_second_next;
            peak_buffer <= peak_buffer_next;
            peak_first_position <= peak_first_position_next;
            peak_second_position <= peak_second_position_next;
            peak_buffer_position <= sample_position[9:0];
            peak_first_valid <= peak_first_valid_next;
            peak_second_valid <= peak_second_valid_next;
            peak_buffer_valid <= 1'b1;
        end
    end

    /* Type 0: area; type 1: peak. */
    assign uo_out  = output_data[7:0];
    assign uio_out = {output_data[10:8], 4'b0000, output_kind};
    assign uio_oe  = output_ready ? 8'he1 : 8'he0;

endmodule
