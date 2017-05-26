
`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:
// Design Name:   
// Module Name:   
// Project Name:  
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: 
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module test_collision_module;

  // Inputs
	reg clk;
	reg reset;
	reg in_val;
	reg [7:0] x1;
	reg [7:0] y1;
	reg [7:0] z1;
	reg [7:0] x2;
	reg [7:0] y2;
	reg [7:0] z2;
	
	wire out_val;
	wire [7:0] lineID;
	
	//To count the clocks
	reg [31:0] cnt;
	
  /************************* instances ***********************/
  CollisionDetect u_1(
	.clk(clk), 
	.reset(reset), 
	.in_val(in_val), 
	.x1(x1),
	.y1(y1),
	.z1(z1),
	.x2(x2),
	.y2(y2),
	.z2(z2),
	.out_val(out_val),
	.lineID(lineID)
	);
 
	integer fd_in;
	integer fd_out;
	integer r_cnt;
	
	initial begin
		// Initialize Inputs
		clk = 1'b1;
		reset = 1'b0;
		cnt = 32'h0;
		fd_in = $fopen("gcode_test_3d.txt","r");
		fd_out = $fopen("result.txt","w");
	end
    
	//Build global clock - 100ns 
	always
	begin
		#50 clk <= ~clk;                      
	end

	always @(posedge clk)
	begin
		if (cnt == 32'h7fffffff) begin                       //if not enough, try to use 64-bit reg
			cnt <= 32'd0;
		end
		else
			cnt <= cnt +32'h1;
	end
 
	//Build 1-clock reset signal
	always @(posedge clk)
	begin
		if (cnt == 32'd2)
			reset <= 1'b1;
		else
			reset <= 1'b0;
	end	
	  
	always @(posedge clk)  
	begin
		if (cnt>32'd2 && cnt[0]) begin
			r_cnt = $fscanf(fd_in,"%d %d %d %d %d %d",x1,y1,z1,x2,y2,z2);
			if (r_cnt==32'h1)
				in_val = 1'b0; //  in this case, we reached the last row(pair of voxel coordinate) of the file
			else
				in_val = 1'b1;
		end
		else
			in_val = 1'b0;
	end
	 
	//record results
	always @(posedge clk)
	begin
		if (out_val) begin
			$fwrite(fd_out,"%d\n",lineID);
		end
	end	
      
endmodule
