`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:   
// Design Name: 
// Module Name:    collisiondetect-3d-opt 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module CollisionDetect(
    input clk,
    input reset,
    input in_val,
    input [7:0] x1,
    input [7:0] y1,
    input [7:0] z1,
    input [7:0] x2,
    input [7:0] y2,
    input [7:0] z2,
    output wire out_val,
    output wire [7:0] lineID
    );
    //Creating the memory module
    reg [7:0] x1_mem[63:0];
    reg [7:0] y1_mem[63:0];
    reg [7:0] z1_mem[63:0];
    reg [7:0] x2_mem[63:0];
    reg [7:0] y2_mem[63:0];
    reg [7:0] z2_mem[63:0];

    reg [63:0] invalid;

    reg [7:0] total;  // # of total lines
    reg [7:0] line_idx, idx_i, idx_j;
    reg [7:0] linecount;  // # of valid lines
    reg detecting, done;

    reg [7:0] null_count;

    reg found;
    wire inter;

    reg [95:0] p1, p2, q1, q2;

    always @(posedge clk) begin
        p1 = {0, 0, 0};
        p2 = {1, 1, 0};
        q1 = {2, 2, 0};
        q2 = {3, 3, 0};

        if (reset) begin
            // reset
            total = 0;
            null_count = 0;
            idx_i = 0;
            idx_j = 1;  // compare with [0..j-1], i < j
            invalid = 64'b0;
            done = 0;
            detecting = 0;
        end else if (in_val) begin
            x1_mem[total] = x1; //store x1 in memory module
            y1_mem[total] = y1; //store y1 in memory module
            z1_mem[total] = z1; //store z1 in memory module
            x2_mem[total] = x2; //store x2 in memory module
            y2_mem[total] = y2; //store y2 in memory module
            z2_mem[total] = z2; //store z2 in memory module
            total = (total < 63) ? total + 1 : total;
            null_count = 0;
        end else if (null_count >= 16 && !done) begin
            detecting = 1;
        end else if (!detecting && !done) begin
            null_count = null_count + 1;
        end

        if (detecting && !done) begin
            if (inter) begin  // should be judged first
                // invalid
                invalid[idx_j] = 1;
                idx_j = idx_j + 1;
                idx_i = 0;
            end else if (idx_j >= total) begin
                done = 1;
                detecting = 0;
                idx_i = 0;
                idx_j = 0;
            end else if (idx_i == idx_j || invalid[idx_j]) begin
                idx_j = idx_j + 1;
                idx_i = 0;
            end else if (invalid[idx_i]) begin
                idx_i = idx_i + 1;
            end else begin
                // valid line[i] and line[j]
                // compare line[i] && line[j]

                // $display("I: %d, J: %d\n", idx_i, idx_j);
                // p1 = {{{24{x1_mem[idx_i][7]}}, x1_mem[idx_i]}, {{24{y1_mem[idx_i][7]}}, y1_mem[idx_i]}, {{24{z1_mem[idx_i][7]}}, z1_mem[idx_i]}};
                // p2 = {{{24{x2_mem[idx_i][7]}}, x2_mem[idx_i]}, {{24{y2_mem[idx_i][7]}}, y2_mem[idx_i]}, {{24{z2_mem[idx_i][7]}}, z2_mem[idx_i]}};
                // q1 = {{{24{x1_mem[idx_j][7]}}, x1_mem[idx_j]}, {{24{y1_mem[idx_j][7]}}, y1_mem[idx_j]}, {{24{z1_mem[idx_j][7]}}, z1_mem[idx_j]}};
                // q2 = {{{24{x2_mem[idx_j][7]}}, x2_mem[idx_j]}, {{24{y2_mem[idx_j][7]}}, y2_mem[idx_j]}, {{24{z2_mem[idx_j][7]}}, z2_mem[idx_j]}};
                p1 = {{24'b0, x1_mem[idx_i]}, {24'b0, y1_mem[idx_i]}, {24'b0, z1_mem[idx_i]}};
                p2 = {{24'b0, x2_mem[idx_i]}, {24'b0, y2_mem[idx_i]}, {24'b0, z2_mem[idx_i]}};
                q1 = {{24'b0, x1_mem[idx_j]}, {24'b0, y1_mem[idx_j]}, {24'b0, z1_mem[idx_j]}};
                q2 = {{24'b0, x2_mem[idx_j]}, {24'b0, y2_mem[idx_j]}, {24'b0, z2_mem[idx_j]}};
                // $display("inter: %d, p1: %x, p2: %x, q1: %x, q2: %x\n", inter, p1, p2, q1, q2);

                found = inter;
                line_idx = idx_j + 1;
                idx_i = idx_i + 1;
                // if (inter) begin
                // end else begin
                //     idx_i = idx_i + 1;
                // end
            end
        end else begin
            found = 0;
            line_idx = -1;
        end
    end

    DetectTwoLines u_1(p1, p2, q1, q2, inter);

    assign lineID = line_idx;
    assign out_val = inter;
endmodule

module DetectTwoLines(
    input wire [95:0] p1,
    input wire [95:0] p2,
    input wire [95:0] q1,
    input wire [95:0] q2,
    output reg intersect
    );
    wire [95:0] p1p2, q1q2, cross;
    wire [3:0] on_seg;
    wire inter;

    vector_v u_1(p1, p2, p1p2);
    vector_v u_2(q1, q2, q1q2);

    on_segment u_3(p1, p2, q1, on_seg[0]);
    on_segment u_4(p1, p2, q2, on_seg[1]);
    on_segment u_5(q1, q2, p1, on_seg[2]);
    on_segment u_6(q1, q2, p2, on_seg[3]);

    cross_product u_7(p1p2, q1q2, cross);

    is_intersect u_8(p1, p2, q1, q2, inter);

    always @* begin
        // $display("on_seg: %x, inter: %d\n", on_seg, inter);
        // $display("p1: %x, p2: %x, q1: %x, q2: %x\n", p1, p2, q1, q2);
        if (on_seg) begin
            intersect = 1'b1;
        end else if (cross == 96'b0) begin
            intersect = 1'b0;
        end else begin
            intersect = inter;
        end
    end
endmodule

module max2(
    input wire [31:0] a,
    input wire [31:0] b,
    output wire [31:0] o
    );
    assign o = (a > b) ? a : b;
endmodule

module min2(
    input wire [31:0] a,
    input wire [31:0] b,
    output wire [31:0] o
    );
    assign o = (a < b) ? a : b;
endmodule

module vector_p(
    input wire [31:0] x1,
    input wire [31:0] y1,
    input wire [31:0] z1,
    input wire [31:0] x2,
    input wire [31:0] y2,
    input wire [31:0] z2,
    output wire [95:0] p
    );
    assign p = {x2 - x1, y2 - y1, z2 - z1};
endmodule

module vector_v(
    input wire [95:0] p1,
    input wire [95:0] p2,
    output wire [95:0] v
    );
    vector_p u_1(p1[95:64], p1[63:32], p1[31:0], 
             p2[95:64], p2[63:32], p2[31:0], 
             v);
endmodule

module cross_product(
    input wire [95:0] v1, 
    input wire [95:0] v2,
    output reg [95:0] o
    );
    reg signed [31:0] x1, y1, z1;
    reg signed [31:0] x2, y2, z2;
    reg signed [31:0] x, y, z;

    always @* begin
        x1 = v1[95:64];
        y1 = v1[63:32];
        z1 = v1[31:0];

        x2 = v2[95:64];
        y2 = v2[63:32];
        z2 = v2[31:0];

        x = y1 * z2 - z1 * y2;
        y = x2 * z1 - x1 * z2;
        z = x1 * y2 - y1 * x2;
        o = {x, y, z};
        // $display("v1: %x, v2: %x, x: %x, y: %x, z: %x o: %x\n", v1, v2, x, y, z, o);
    end
endmodule

module dot_product(
    input wire [95:0] v1,
    input wire [95:0] v2,
    output reg [63:0] result
    );
    reg signed [31:0] x1, y1, z1;
    reg signed [31:0] x2, y2, z2;

    always @* begin
        x1 = v1[95:64];
        y1 = v1[63:32];
        z1 = v1[31:0];

        x2 = v2[95:64];
        y2 = v2[63:32];
        z2 = v2[31:0];

        result = x1 * x2 + y1 * y2 + z1 * z2;
    end
endmodule

module hybrid_product(
    input wire [95:0] v1,
    input wire [95:0] v2,
    input wire [95:0] v3,
    output wire [63:0] result
    );
    wire [95:0] v12;
    cross_product u_1(v1, v2, v12);
    dot_product u_2(v12, v3, result);
endmodule

module add_vec(
    input wire [95:0] p1,
    input wire [95:0] v,
    output reg [95:0] o
    );
    reg signed [31:0] x1, y1, z1;
    reg signed [31:0] x2, y2, z2;
    reg signed [31:0] x, y, z;

    always @* begin
        x1 = p1[95:64];
        y1 = p1[63:32];
        z1 = p1[31:0];

        x2 = v[95:64];
        y2 = v[63:32];
        z2 = v[31:0];

        x = x2 + x1;
        y = y2 + y1;
        z = z2 + z1;

        o = {x, y, z};
    end
endmodule

module is_vertical(
    input wire [95:0] p1,
    input wire [95:0] p2,
    input wire [95:0] q1,
    input wire [95:0] q2,
    output wire o
    );
    wire [95:0] v1, v2;
    wire [31:0] result;

    vector_v u_1(p1, p2, v1);
    vector_v u_2(q1, q2, v2);
    dot_product u_3(v1, v2, result);

    assign o = result ? 1'b0 : 1'b1;
endmodule

module on_segment(
    input wire [95:0] p1,
    input wire [95:0] p2,
    input wire [95:0] q,
    output reg o
    );
    wire [95:0] p1p2, p1q, cross;
    reg signed [31:0] x1, y1, z1;
    reg signed [31:0] x2, y2, z2;
    reg signed [31:0] x, y, z;

    vector_v u_1(p1, p2, p1p2);
    vector_v u_2(p1, q, p1q);
    cross_product u_3(p1p2, p1q, cross);

    always @* begin
        x1 = p1[95:64];
        y1 = p1[63:32];
        z1 = p1[31:0];

        x2 = p2[95:64];
        y2 = p2[63:32];
        z2 = p2[31:0];

        x = q[95:64];
        y = q[63:32];
        z = q[31:0];

        if (cross != 96'b0) begin
            o = 1'b0;  // False
        end else begin
            if (x <= max(x1, x2) && x >= min(x1, x2) && y <= max(y1, y2) && y >= min(y1, y2) && z <= max(z1, z2) && z >= min(z1, z2))
                o = 1'b1;  // True
            else
                o = 1'b0;
        end
        // $display("cross: %x o: %d\n", cross, o);
    end

    function [31:0] max;
        input [31:0] a, b;
        max = ($signed(a) >= $signed(b)) ? a : b;
    endfunction
     
    function [31:0] min;
        input [31:0] a, b;
        min = ($signed(a) <= $signed(b)) ? a : b;
    endfunction
endmodule

module is_intersect(
    input wire [95:0] p1,
    input wire [95:0] p2,
    input wire [95:0] q1,
    input wire [95:0] q2,
    output wire o
    );
    wire [95:0] p1p2, q1q2, p1q1, p1q2, q1p1, q1p2;
    wire [95:0] vz, z, z1, p1z, q1z1;
    wire [63:0] hybrid[4:0];
    wire [63:0] delta1, delta2;

    vector_v u_1(p1, p2, p1p2);
    vector_v u_2(q1, q2, q1q2);
    vector_v u_3(p1, q1, p1q1);
    vector_v u_4(p1, q2, p1q2);
    vector_v u_5(q1, p1, q1p1);
    vector_v u_6(q1, p2, q1p2);

    hybrid_product u_7(p1p2, q1q2, p1q1, hybrid[0]);

    cross_product u_8(p1p2, q1q2, vz);
    add_vec u_9(p1, vz, z);
    add_vec u_10(q1, vz, z1);
    vector_v u_11(p1, z, p1z);
    vector_v u_12(q1, z1, q1z1);

    hybrid_product u_13(p1p2, p1q1, p1z, hybrid[1]);
    hybrid_product u_14(p1p2, p1q2, p1z, hybrid[2]);
    hybrid_product u_15(q1q2, q1p1, q1z1, hybrid[3]);
    hybrid_product u_16(q1q2, q1p2, q1z1, hybrid[4]);

    assign delta1 = hybrid[1] * hybrid[2];
    assign delta2 = hybrid[3] * hybrid[4];

    assign o = (hybrid[0] != 64'b0) ? 1'b0 : (($signed(delta1) <= 0 && $signed(delta2) <= 0) ? 1'b1 : 1'b0);

    // always @* begin
    //     // $display("h0: %d, h1: %d, h2: %d, h3: %d, h4: %d\n", $signed(hybrid[0]), $signed(hybrid[1]), $signed(hybrid[2]), $signed(hybrid[3]), $signed(hybrid[4]));
    //     // $display("delta1: %d, delta2: %d\n", $signed(delta1), $signed(delta2));
    //     if (hybrid[0] != 64'b0) begin
    //         o = 1'b0;
    //     end else if ($signed(delta1) <= 0 && $signed(delta2) <= 0) begin
    //         o = 1'b1;
    //     end else begin
    //         o = 1'b0;
    //     end
    // end
endmodule
