//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/20/2021 12:58:25 PM
// Design Name: 
// Module Name: iccm_controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: s
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module prog_uart_controller (
	clk_i,
	rst_ni,
	rx_dv_i,
	rx_byte_i,
	we_o,
	addr_o,
	wdata_o,
	reset_o
);
	input wire clk_i;
	input wire rst_ni;
	input wire rx_dv_i;
	input wire [7:0] rx_byte_i;
	output wire we_o;
	output wire [13:0] addr_o;
	output wire [63:0] wdata_o;
	output wire reset_o;
	reg [1:0] ctrl_fsm_cs;
	reg [1:0] ctrl_fsm_ns;
	wire [7:0] rx_byte_d;
	reg [7:0] rx_byte_q0;
	reg [7:0] rx_byte_q1;
	reg [7:0] rx_byte_q2;
	reg [7:0] rx_byte_q3;
	reg [7:0] rx_byte_q4;
	reg [7:0] rx_byte_q5;
	reg [7:0] rx_byte_q6;
	reg [7:0] rx_byte_q7;
	reg we_q;
	reg we_d;
	reg [13:0] addr_q;
	reg [13:0] addr_d;
	reg reset_q;
	reg reset_d;
	reg [2:0] byte_count;
	localparam [1:0] DONE = 3;
	localparam [1:0] LOAD = 1;
	localparam [1:0] PROG = 2;
	localparam [1:0] RESET = 0;
	always @(*) begin
		we_d = we_q;
		addr_d = addr_q;
		reset_d = reset_q;
		ctrl_fsm_ns = ctrl_fsm_cs;
		case (ctrl_fsm_cs)
			RESET: begin
				we_d = 1'b0;
				reset_d = 1'b0;
				if (rx_dv_i)
					ctrl_fsm_ns = LOAD;
				else
					ctrl_fsm_ns = RESET;
			end
			LOAD:
				if (((byte_count == 3'b111) /*&& (rx_byte_q2 != 8'h0f)) && (rx_byte_d != 8'hff)*/)) begin
					we_d = 1'b1;
					ctrl_fsm_ns = PROG;
				end
				else
					ctrl_fsm_ns = DONE;
			PROG: begin
				we_d = 1'b0;
				ctrl_fsm_ns = DONE;
			end
			DONE:
				if (wdata_o == 64'h0000000000000fff) begin
					ctrl_fsm_ns = DONE;
					reset_d = 1'b1;
				end
				else if (rx_dv_i)
					ctrl_fsm_ns = LOAD;
				else
					ctrl_fsm_ns = DONE;
			default: ctrl_fsm_ns = RESET;
		endcase
	end
	assign rx_byte_d = rx_byte_i;
	assign we_o = we_q;
	assign addr_o = addr_q;
	assign wdata_o = {rx_byte_q7, rx_byte_q6, rx_byte_q5, rx_byte_q4, rx_byte_q3, rx_byte_q2, rx_byte_q1, rx_byte_q0};
	assign reset_o = reset_q;
	always @(posedge clk_i or negedge rst_ni)
		if (!rst_ni) begin
			we_q <= 1'b0;
			addr_q <= 14'b00000000000000;
			rx_byte_q0 <= 8'b00000000;
			rx_byte_q1 <= 8'b00000000;
			rx_byte_q2 <= 8'b00000000;
			rx_byte_q3 <= 8'b00000000;
			rx_byte_q4 <= 8'b00000000;
			rx_byte_q5 <= 8'b00000000;
			rx_byte_q6 <= 8'b00000000;
			rx_byte_q7 <= 8'b00000000;
			reset_q <= 1'b0;
			byte_count <= 3'b00;
			ctrl_fsm_cs <= RESET;
		end
		else begin
			we_q <= we_d;
			if (ctrl_fsm_cs == LOAD) begin
				if (byte_count == 3'b000) begin
					rx_byte_q0 <= rx_byte_d;
					byte_count <= 3'b001;
				end
				else if (byte_count == 3'b001) begin
					rx_byte_q1 <= rx_byte_d;
					byte_count <= 3'b010;
				end
				else if (byte_count == 3'b010) begin
					rx_byte_q2 <= rx_byte_d;
					byte_count <= 3'b011;
				end
		        else if (byte_count == 2'b011) begin
                    rx_byte_q3 <= rx_byte_d;
                    byte_count <= 3'b100;
                end
		        else if (byte_count == 3'b100) begin
                    rx_byte_q4 <= rx_byte_d;
                    byte_count <= 3'b101;
                end
		        else if (byte_count == 3'b101) begin
                    rx_byte_q5 <= rx_byte_d;
                    byte_count <= 3'b110;
                end
		        else if (byte_count == 3'b110) begin
                    rx_byte_q6 <= rx_byte_d;
                    byte_count <= 3'b111;
                end
				else begin
					rx_byte_q7 <= rx_byte_d;
					byte_count <= 3'b000;
				end
				addr_q <= addr_d;
			end
			if (ctrl_fsm_cs == PROG)
				addr_q <= addr_d + 1'b1;
			reset_q <= reset_d;
			ctrl_fsm_cs <= ctrl_fsm_ns;
		end
endmodule