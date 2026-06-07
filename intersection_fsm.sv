module intersection_fsm #(
    parameter ADDR_WIDTH = 12
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // C++ Control Interface
    input  wire                   scene_valid,
    input  wire  [31:0]           scene_ignore_idx,
    output logic                  scene_ready,
    output logic                  scene_hit,
    output logic [31:0]           scene_t,
    output logic [31:0]           scene_hit_idx,

    // BRAM Fetching Interface
    input  wire  [31:0]           bram_rdata,
    output logic [ADDR_WIDTH-1:0] bram_addr,
    output logic                  bram_read_active,

    // Core Intersector Interface
    input  wire                   isect_ready,
    input  wire                   isect_hit,
    input  wire  [31:0]           isect_t,
    output logic                  isect_valid,
    output logic [31:0]           v0_x, v0_y, v0_z,
    output logic [31:0]           v1_x, v1_y, v1_z,
    output logic [31:0]           v2_x, v2_y, v2_z
);

    typedef enum logic [3:0] {
        IDLE,
        FETCH_REQ,
        FETCH_WAIT,
        FETCH_CAPTURE,
        START_ISECT,
        WAIT_ISECT,
        DONE
    } state_t;

    state_t state;

    logic [31:0] current_tri;
    logic [3:0]  word_idx;
    logic [31:0] closest_t;
    logic [31:0] closest_idx;
    logic        any_hit;

    // Prefetch registers for the next triangle
    logic [31:0] next_tri;
    logic [3:0]  next_word_idx;
    logic        next_valid;
    logic        prefetch_pending;

    logic [31:0] next_v0_x, next_v0_y, next_v0_z;
    logic [31:0] next_v1_x, next_v1_y, next_v1_z;
    logic [31:0] next_v2_x, next_v2_y, next_v2_z;

    localparam MAX_T = 32'h7FFFFFFF;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;

            scene_ready <= 0;
            scene_hit <= 0;
            scene_t <= 0;
            scene_hit_idx <= 0;

            bram_addr <= 0;
            bram_read_active <= 0;

            isect_valid <= 0;

            current_tri <= 0;
            word_idx <= 0;
            closest_t <= MAX_T;
            closest_idx <= 0;
            any_hit <= 0;

            next_tri <= 0;
            next_word_idx <= 0;
            next_valid <= 0;
            prefetch_pending <= 0;

            v0_x <= 0; v0_y <= 0; v0_z <= 0;
            v1_x <= 0; v1_y <= 0; v1_z <= 0;
            v2_x <= 0; v2_y <= 0; v2_z <= 0;

            next_v0_x <= 0; next_v0_y <= 0; next_v0_z <= 0;
            next_v1_x <= 0; next_v1_y <= 0; next_v1_z <= 0;
            next_v2_x <= 0; next_v2_y <= 0; next_v2_z <= 0;
        end else begin
            case (state)
                IDLE: begin
                    scene_ready <= 0;
                    scene_hit <= 0;
                    scene_t <= 0;
                    scene_hit_idx <= 0;
                    bram_read_active <= 0;
                    isect_valid <= 0;

                    next_valid <= 0;
                    prefetch_pending <= 0;

                    if (scene_valid) begin
                        current_tri <= 0;
                        word_idx <= 0;
                        closest_t <= MAX_T;
                        any_hit <= 0;
                        closest_idx <= 0;
                        bram_read_active <= 1;
                        state <= FETCH_REQ;
                    end
                end

                FETCH_REQ: begin
                    bram_addr <= (current_tri << 4) + word_idx;
                    state <= FETCH_WAIT;
                end

                FETCH_WAIT: begin
                    state <= FETCH_CAPTURE;
                end

                FETCH_CAPTURE: begin
                    // END token on first word
                    if (word_idx == 0 && bram_rdata == 32'hFFFFFFFF) begin
                        state <= DONE;
                    end else begin
                        case (word_idx)
                            4'd0: v0_x <= bram_rdata;
                            4'd1: v0_y <= bram_rdata;
                            4'd2: v0_z <= bram_rdata;
                            4'd3: v1_x <= bram_rdata;
                            4'd4: v1_y <= bram_rdata;
                            4'd5: v1_z <= bram_rdata;
                            4'd6: v2_x <= bram_rdata;
                            4'd7: v2_y <= bram_rdata;
                            4'd8: v2_z <= bram_rdata;
                        endcase

                        if (word_idx == 4'd8) begin
                            word_idx <= 0;
                            if (current_tri == scene_ignore_idx) begin
                                current_tri <= current_tri + 1;
                                state <= FETCH_REQ;
                            end else begin
                                state <= START_ISECT;
                            end
                        end else begin
                            word_idx <= word_idx + 1;
                            state <= FETCH_REQ;
                        end
                    end
                end

                START_ISECT: begin
                    isect_valid <= 1;

                    // initialise next-triangle prefetch
                    next_tri <= current_tri + 1;
                    next_word_idx <= 0;
                    next_valid <= 0;
                    prefetch_pending <= 1;

                    state <= WAIT_ISECT;
                end

                WAIT_ISECT: begin
                    isect_valid <= 0;

                    // Continue prefetch while waiting for the current intersection result.
                    // BRAM is synchronous, so each word takes two phases:
                    //   cycle A: issue address
                    //   cycle B: capture returned data
                    if (prefetch_pending && !next_valid) begin
                        if (next_word_idx[0] == 1'b0) begin
                            // even step: issue address
                            bram_read_active <= 1;
                            bram_addr <= (next_tri << 4) + (next_word_idx >> 1);
                            next_word_idx <= next_word_idx + 1;
                        end else begin
                            // odd step: capture data
                            case (next_word_idx >> 1)
                                4'd0: next_v0_x <= bram_rdata;
                                4'd1: next_v0_y <= bram_rdata;
                                4'd2: next_v0_z <= bram_rdata;
                                4'd3: next_v1_x <= bram_rdata;
                                4'd4: next_v1_y <= bram_rdata;
                                4'd5: next_v1_z <= bram_rdata;
                                4'd6: next_v2_x <= bram_rdata;
                                4'd7: next_v2_y <= bram_rdata;
                                4'd8: next_v2_z <= bram_rdata;
                            endcase

                            if ((next_word_idx >> 1) == 4'd8) begin
                                next_valid <= 1;
                                prefetch_pending <= 0;
                            end else begin
                                next_word_idx <= next_word_idx + 1;
                            end
                        end
                    end

                    if (isect_ready) begin
                        if (isect_hit && $signed(isect_t) > $signed(32'd10) && $signed(isect_t) < $signed(closest_t)) begin
                            closest_t <= isect_t;
                            closest_idx <= current_tri;
                            any_hit <= 1;
                        end

                        // Move to next triangle
                        current_tri <= current_tri + 1;

                        if (next_valid) begin
                            // Next triangle already prefetched, so use it immediately
                            if (next_tri == scene_ignore_idx) begin
                                current_tri <= next_tri;
                                word_idx <= 0;
                                next_valid <= 0;
                                prefetch_pending <= 0;
                                state <= FETCH_REQ;
                            end else begin
                                v0_x <= next_v0_x; v0_y <= next_v0_y; v0_z <= next_v0_z;
                                v1_x <= next_v1_x; v1_y <= next_v1_y; v1_z <= next_v1_z;
                                v2_x <= next_v2_x; v2_y <= next_v2_y; v2_z <= next_v2_z;

                                current_tri <= next_tri;
                                next_valid <= 0;
                                prefetch_pending <= 0;
                                state <= START_ISECT;
                            end
                        end else begin
                            // Prefetch not finished yet, fall back to normal fetch
                            state <= FETCH_REQ;
                        end
                    end
                end

                DONE: begin
                    scene_ready <= 1;
                    scene_hit <= any_hit;
                    scene_t <= closest_t;
                    scene_hit_idx <= closest_idx;
                    bram_read_active <= 0;
                    isect_valid <= 0;
                    next_valid <= 0;
                    prefetch_pending <= 0;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule
