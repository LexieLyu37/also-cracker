`timescale 1ns/1ps

module ray_triangle_intersect
#(
    parameter FRAC = 16,                  // fractional bits (Q16.16)
    parameter logic signed [31:0] EPSILON = 32'sd1
) (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         valid,           // pulse high to start

    // Ray – Q16.16
    input  logic signed [31:0] ray_origin_x, ray_origin_y, ray_origin_z,
    input  logic signed [31:0] ray_dir_x,    ray_dir_y,    ray_dir_z,

    // Triangle vertices – Q16.16
    input  logic signed [31:0] v0_x, v0_y, v0_z,
    input  logic signed [31:0] v1_x, v1_y, v1_z,
    input  logic signed [31:0] v2_x, v2_y, v2_z,

    // Outputs
    output logic         hit,
    output logic signed [31:0] t,
    output logic         ready
);

    // FSM States
    typedef enum logic [3:0] {
        S_IDLE,
        S_H_CROSS,
        S_A_DOT,
        S_U_DOT,
        S_Q_CROSS,
        S_V_DOT,
        S_T_DOT,
        S_DIV_START,
        S_DIV_STEP,
        S_DONE
    } state_t;

    state_t state;

    // Internal registers
    logic signed [31:0] rd_x_r,  rd_y_r,  rd_z_r;
    logic signed [31:0] edge1_x, edge1_y, edge1_z;
    logic signed [31:0] edge2_x, edge2_y, edge2_z;
    logic signed [31:0] s_x,     s_y,     s_z;
    logic signed [31:0] h_x, h_y, h_z;
    logic signed [31:0] q_x, q_y, q_z;
    logic signed [31:0] a, u, v, t_num;

    // Shared vector math units
    logic signed [31:0] vec_a_x, vec_a_y, vec_a_z;
    logic signed [31:0] vec_b_x, vec_b_y, vec_b_z;
    logic signed [31:0] cross_out_x, cross_out_y, cross_out_z;
    logic signed [31:0] dot_out;

    function automatic logic signed [31:0] mul_q16(input logic signed [31:0] a_in, b_in);
        logic signed [63:0] a64, b64, prod;
        begin
            a64 = a_in;
            b64 = b_in;
            prod = a64 * b64;
            return prod >>> FRAC;
        end
    endfunction

    // Shared cross product
    assign cross_out_x = mul_q16(vec_a_y, vec_b_z) - mul_q16(vec_a_z, vec_b_y);
    assign cross_out_y = mul_q16(vec_a_z, vec_b_x) - mul_q16(vec_a_x, vec_b_z);
    assign cross_out_z = mul_q16(vec_a_x, vec_b_y) - mul_q16(vec_a_y, vec_b_x);

    // Shared dot product
    assign dot_out = mul_q16(vec_a_x, vec_b_x) +
                     mul_q16(vec_a_y, vec_b_y) +
                     mul_q16(vec_a_z, vec_b_z);

    // Select math inputs depending on state
    always_comb begin
        vec_a_x = 32'sd0; vec_a_y = 32'sd0; vec_a_z = 32'sd0;
        vec_b_x = 32'sd0; vec_b_y = 32'sd0; vec_b_z = 32'sd0;

        case (state)
            S_H_CROSS: begin
                vec_a_x = rd_x_r;  vec_a_y = rd_y_r;  vec_a_z = rd_z_r;
                vec_b_x = edge2_x; vec_b_y = edge2_y; vec_b_z = edge2_z;
            end

            S_A_DOT: begin
                vec_a_x = edge1_x; vec_a_y = edge1_y; vec_a_z = edge1_z;
                vec_b_x = h_x;     vec_b_y = h_y;     vec_b_z = h_z;
            end

            S_U_DOT: begin
                vec_a_x = s_x;     vec_a_y = s_y;     vec_a_z = s_z;
                vec_b_x = h_x;     vec_b_y = h_y;     vec_b_z = h_z;
            end

            S_Q_CROSS: begin
                vec_a_x = s_x;     vec_a_y = s_y;     vec_a_z = s_z;
                vec_b_x = edge1_x; vec_b_y = edge1_y; vec_b_z = edge1_z;
            end

            S_V_DOT: begin
                vec_a_x = rd_x_r;  vec_a_y = rd_y_r;  vec_a_z = rd_z_r;
                vec_b_x = q_x;     vec_b_y = q_y;     vec_b_z = q_z;
            end

            S_T_DOT: begin
                vec_a_x = edge2_x; vec_a_y = edge2_y; vec_a_z = edge2_z;
                vec_b_x = q_x;     vec_b_y = q_y;     vec_b_z = q_z;
            end

            default: ;
        endcase
    end

    // Early-exit checks
    logic a_is_zero;
    logic u_ok;
    logic v_ok;

    assign a_is_zero = (dot_out > -EPSILON) && (dot_out < EPSILON);

    assign u_ok = ((a > 0) && (dot_out >= 0) && (dot_out <= a)) ||
                  ((a < 0) && (dot_out <= 0) && (dot_out >= a));

    assign v_ok = ((a > 0) && (dot_out >= 0) && (dot_out <= a) && ((u + dot_out) <= a)) ||
                  ((a < 0) && (dot_out <= 0) && (dot_out >= a) && ((u + dot_out) >= a));

    // Sequential divider registers
    logic [63:0] div_num;
    logic [63:0] div_den;
    logic [63:0] div_rem;
    logic [63:0] div_quot;
    logic [6:0]  div_count;
    logic        div_sign;

    logic [63:0] next_rem;
    assign next_rem = {div_rem[62:0], div_num[63]};

    logic signed [31:0] final_t;
    assign final_t = div_sign ? -$signed(div_quot[31:0]) : $signed(div_quot[31:0]);

    // Main FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;

            ready <= 1'b0;
            hit   <= 1'b0;
            t     <= 32'sd0;

            rd_x_r  <= 32'sd0; rd_y_r  <= 32'sd0; rd_z_r  <= 32'sd0;
            edge1_x <= 32'sd0; edge1_y <= 32'sd0; edge1_z <= 32'sd0;
            edge2_x <= 32'sd0; edge2_y <= 32'sd0; edge2_z <= 32'sd0;
            s_x     <= 32'sd0; s_y     <= 32'sd0; s_z     <= 32'sd0;
            h_x     <= 32'sd0; h_y     <= 32'sd0; h_z     <= 32'sd0;
            q_x     <= 32'sd0; q_y     <= 32'sd0; q_z     <= 32'sd0;
            a       <= 32'sd0; u       <= 32'sd0; v       <= 32'sd0; t_num <= 32'sd0;

            div_num   <= 64'd0;
            div_den   <= 64'd0;
            div_rem   <= 64'd0;
            div_quot  <= 64'd0;
            div_count <= 7'd0;
            div_sign  <= 1'b0;

        end else begin
            case (state)

                S_IDLE: begin
                    ready <= 1'b0;
                    hit   <= 1'b0;
                    t     <= 32'sd0;

                    if (valid) begin
                        // Latch ray and precompute linear vectors
                        rd_x_r  <= ray_dir_x;
                        rd_y_r  <= ray_dir_y;
                        rd_z_r  <= ray_dir_z;

                        edge1_x <= v1_x - v0_x;
                        edge1_y <= v1_y - v0_y;
                        edge1_z <= v1_z - v0_z;

                        edge2_x <= v2_x - v0_x;
                        edge2_y <= v2_y - v0_y;
                        edge2_z <= v2_z - v0_z;

                        s_x <= ray_origin_x - v0_x;
                        s_y <= ray_origin_y - v0_y;
                        s_z <= ray_origin_z - v0_z;

                        state <= S_H_CROSS;
                    end
                end

                S_H_CROSS: begin
                    h_x <= cross_out_x;
                    h_y <= cross_out_y;
                    h_z <= cross_out_z;
                    state <= S_A_DOT;
                end

                S_A_DOT: begin
                    a <= dot_out;
                    if (a_is_zero) begin
                        hit <= 1'b0;
                        t   <= 32'sd0;
                        state <= S_DONE;
                    end else begin
                        state <= S_U_DOT;
                    end
                end

                S_U_DOT: begin
                    u <= dot_out;
                    if (u_ok) begin
                        state <= S_Q_CROSS;
                    end else begin
                        hit <= 1'b0;
                        t   <= 32'sd0;
                        state <= S_DONE;
                    end
                end

                S_Q_CROSS: begin
                    q_x <= cross_out_x;
                    q_y <= cross_out_y;
                    q_z <= cross_out_z;
                    state <= S_V_DOT;
                end

                S_V_DOT: begin
                    v <= dot_out;
                    if (v_ok) begin
                        state <= S_T_DOT;
                    end else begin
                        hit <= 1'b0;
                        t   <= 32'sd0;
                        state <= S_DONE;
                    end
                end

                S_T_DOT: begin
                    t_num <= dot_out;
                    state <= S_DIV_START;
                end

                S_DIV_START: begin
                    // Prepare sequential divider
                    div_num   <= {32'd0, (t_num[31] ? -t_num : t_num)} << FRAC;
                    div_den   <= {32'd0, (a[31] ? -a : a)};
                    div_rem   <= 64'd0;
                    div_quot  <= 64'd0;
                    div_count <= 7'd50;
                    div_sign  <= t_num[31] ^ a[31];

                    // If denominator somehow becomes zero, just fail cleanly
                    if (a == 0) begin
                        hit <= 1'b0;
                        t   <= 32'sd0;
                        state <= S_DONE;
                    end else begin
                        state <= S_DIV_STEP;
                    end
                end

                S_DIV_STEP: begin
                    if (div_count != 0) begin
                        div_num <= {div_num[62:0], 1'b0};

                        if (next_rem >= div_den) begin
                            div_rem  <= next_rem - div_den;
                            div_quot <= {div_quot[62:0], 1'b1};
                        end else begin
                            div_rem  <= next_rem;
                            div_quot <= {div_quot[62:0], 1'b0};
                        end

                        div_count <= div_count - 1'b1;
                    end else begin
                        t <= final_t;

                        if (final_t > EPSILON) begin
                            hit <= 1'b1;
                        end else begin
                            hit <= 1'b0;
                        end

                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    ready <= 1'b1;
                    state <= S_IDLE;
                end

                default: begin
                    ready <= 1'b0;
                    hit   <= 1'b0;
                    t     <= 32'sd0;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
