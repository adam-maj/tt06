`default_nettype none
`timescale 1ns/1ns
module controller (
	clk,
	reset,
	consumer_read_valid,
	consumer_read_address,
	consumer_read_ready,
	consumer_read_data,
	consumer_write_valid,
	consumer_write_address,
	consumer_write_data,
	consumer_write_ready,
	mem_read_valid,
	mem_read_address,
	mem_read_ready,
	mem_read_data,
	mem_write_valid,
	mem_write_address,
	mem_write_data,
	mem_write_ready
);
	parameter ADDR_BITS = 8;
	parameter DATA_BITS = 16;
	parameter NUM_CONSUMERS = 4;
	input wire clk;
	input wire reset;
	input wire [NUM_CONSUMERS - 1:0] consumer_read_valid;
	input wire [(NUM_CONSUMERS * ADDR_BITS) - 1:0] consumer_read_address;
	output wire [NUM_CONSUMERS - 1:0] consumer_read_ready;
	output wire [(NUM_CONSUMERS * DATA_BITS) - 1:0] consumer_read_data;
	input wire [NUM_CONSUMERS - 1:0] consumer_write_valid;
	input wire [(NUM_CONSUMERS * ADDR_BITS) - 1:0] consumer_write_address;
	input wire [(NUM_CONSUMERS * DATA_BITS) - 1:0] consumer_write_data;
	output wire [NUM_CONSUMERS - 1:0] consumer_write_ready;
	output reg mem_read_valid;
	output reg [ADDR_BITS - 1:0] mem_read_address;
	input wire mem_read_ready;
	input wire [DATA_BITS - 1:0] mem_read_data;
	output reg mem_write_valid;
	output reg [ADDR_BITS - 1:0] mem_write_address;
	output reg [DATA_BITS - 1:0] mem_write_data;
	input wire mem_write_ready;
	reg [NUM_CONSUMERS - 1:0] request_pending;
	localparam IDLE = 2'b00;
	localparam WAITING = 2'b01;
	localparam RELAYING = 2'b10;
	reg [1:0] state = IDLE;
	reg [$clog2(NUM_CONSUMERS) - 1:0] current_consumer;
	reg [NUM_CONSUMERS - 1:0] response_valid;
	reg [DATA_BITS - 1:0] response_data [NUM_CONSUMERS - 1:0];
	always @(*) request_pending = (consumer_read_valid | consumer_write_valid) & ~(consumer_read_ready | consumer_write_ready);
	always @(posedge clk)
		if (reset) begin
			current_consumer <= 0;
			state <= IDLE;
			mem_read_valid <= 0;
			mem_read_address <= 0;
			mem_write_valid <= 0;
			mem_write_address <= 0;
			mem_write_data <= 0;
			response_valid <= 0;
			begin : sv2v_autoblock_1
				reg signed [31:0] i;
				for (i = 0; i < NUM_CONSUMERS; i = i + 1)
					response_data[i] <= 0;
			end
		end
		else
			case (state)
				IDLE:
					if (request_pending[current_consumer])
						state <= WAITING;
					else
						current_consumer <= (current_consumer == (NUM_CONSUMERS - 1) ? 0 : current_consumer + 1);
				WAITING:
					if (consumer_read_valid[current_consumer]) begin
						if (mem_read_ready) begin
							mem_read_valid <= 0;
							response_valid[current_consumer] <= 1;
							response_data[current_consumer] <= mem_read_data;
							state <= RELAYING;
						end
						else begin
							mem_read_valid <= 1;
							mem_read_address <= consumer_read_address[current_consumer * ADDR_BITS+:ADDR_BITS];
						end
					end
					else if (consumer_write_valid[current_consumer]) begin
						if (mem_write_ready) begin
							mem_write_valid <= 0;
							response_valid[current_consumer] <= 1;
							state <= RELAYING;
						end
						else begin
							mem_write_valid <= 1;
							mem_write_address <= consumer_write_address[current_consumer * ADDR_BITS+:ADDR_BITS];
							mem_write_data <= consumer_write_data[current_consumer * DATA_BITS+:DATA_BITS];
						end
					end
				RELAYING:
					if (!request_pending[current_consumer]) begin
						response_valid[current_consumer] <= 0;
						state <= IDLE;
					end
			endcase
	genvar _gv_i_1;
	generate
		for (_gv_i_1 = 0; _gv_i_1 < NUM_CONSUMERS; _gv_i_1 = _gv_i_1 + 1) begin : genblk1
			localparam i = _gv_i_1;
			assign consumer_read_ready[i] = response_valid[i] & consumer_read_valid[i];
			assign consumer_write_ready[i] = response_valid[i] & consumer_write_valid[i];
			assign consumer_read_data[i * DATA_BITS+:DATA_BITS] = response_data[i];
		end
	endgenerate
endmodule
`default_nettype none
module core (
	clk,
	reset,
	start,
	done,
	block_dim,
	thread_count,
	program_mem_read_valid,
	program_mem_read_address,
	program_mem_read_ready,
	program_mem_read_data,
	data_mem_read_valid,
	data_mem_read_address,
	data_mem_read_ready,
	data_mem_read_data,
	data_mem_write_valid,
	data_mem_write_address,
	data_mem_write_data,
	data_mem_write_ready
);
	parameter DATA_MEM_ADDR_BITS = 8;
	parameter DATA_MEM_DATA_BITS = 8;
	parameter PROGRAM_MEM_ADDR_BITS = 8;
	parameter PROGRAM_MEM_DATA_BITS = 16;
	parameter MAX_WARPS_PER_CORE = 2;
	parameter THREADS_PER_WARP = 2;
	parameter THREAD_ID_BITS = $clog2(THREADS_PER_WARP);
	parameter CORE_ID = 0;
	input wire clk;
	input wire reset;
	input wire start;
	output wire done;
	input wire [7:0] block_dim;
	input wire [7:0] thread_count;
	output reg program_mem_read_valid;
	output reg [PROGRAM_MEM_ADDR_BITS - 1:0] program_mem_read_address;
	input wire program_mem_read_ready;
	input wire [PROGRAM_MEM_DATA_BITS - 1:0] program_mem_read_data;
	output reg [THREADS_PER_WARP - 1:0] data_mem_read_valid;
	output reg [(THREADS_PER_WARP * DATA_MEM_ADDR_BITS) - 1:0] data_mem_read_address;
	input wire [THREADS_PER_WARP - 1:0] data_mem_read_ready;
	input wire [(THREADS_PER_WARP * DATA_MEM_DATA_BITS) - 1:0] data_mem_read_data;
	output reg [THREADS_PER_WARP - 1:0] data_mem_write_valid;
	output reg [(THREADS_PER_WARP * DATA_MEM_ADDR_BITS) - 1:0] data_mem_write_address;
	output reg [(THREADS_PER_WARP * DATA_MEM_DATA_BITS) - 1:0] data_mem_write_data;
	input wire [THREADS_PER_WARP - 1:0] data_mem_write_ready;
	reg [(MAX_WARPS_PER_CORE * 8) - 1:0] warp_pc;
	reg [THREAD_ID_BITS - 1:0] current_warp_id;
	localparam IDLE = 2'b00;
	localparam FETCHING = 2'b01;
	localparam PROCESSING = 2'b10;
	localparam WAITING = 2'b11;
	reg [2:0] core_state;
	reg [2:0] fetcher_state;
	reg [15:0] instruction;
	reg [7:0] rs [THREADS_PER_WARP - 1:0];
	reg [7:0] rt [THREADS_PER_WARP - 1:0];
	reg [7:0] rd [THREADS_PER_WARP - 1:0];
	reg [7:0] alu_out [THREADS_PER_WARP - 1:0];
	reg [(THREADS_PER_WARP * 2) - 1:0] lsu_state;
	reg [7:0] lsu_out [THREADS_PER_WARP - 1:0];
	reg [7:0] next_pc [THREADS_PER_WARP - 1:0];
	reg [3:0] decoded_rd_address;
	reg [3:0] decoded_rs_address;
	reg [3:0] decoded_rt_address;
	reg [2:0] decoded_nzp;
	reg [7:0] decoded_immediate;
	reg decoded_reg_write_enable;
	reg decoded_mem_read_enable;
	reg decoded_mem_write_enable;
	reg decoded_nzp_write_enable;
	reg [1:0] decoded_reg_input_mux;
	reg [1:0] decoded_alu_arithmetic_mux;
	reg decoded_alu_output_mux;
	reg decoded_pc_mux;
	reg decoded_ret;
	wire [1:1] sv2v_tmp_fetcher_instance_mem_read_valid;
	always @(*) program_mem_read_valid = sv2v_tmp_fetcher_instance_mem_read_valid;
	wire [PROGRAM_MEM_ADDR_BITS:1] sv2v_tmp_fetcher_instance_mem_read_address;
	always @(*) program_mem_read_address = sv2v_tmp_fetcher_instance_mem_read_address;
	wire [3:1] sv2v_tmp_fetcher_instance_fetcher_state;
	always @(*) fetcher_state = sv2v_tmp_fetcher_instance_fetcher_state;
	wire [16:1] sv2v_tmp_fetcher_instance_instruction;
	always @(*) instruction = sv2v_tmp_fetcher_instance_instruction;
	fetcher #(
		.PROGRAM_MEM_ADDR_BITS(PROGRAM_MEM_ADDR_BITS),
		.PROGRAM_MEM_DATA_BITS(PROGRAM_MEM_DATA_BITS)
	) fetcher_instance(
		.clk(clk),
		.reset(reset),
		.core_state(core_state),
		.current_pc(warp_pc[current_warp_id * 8+:8]),
		.mem_read_valid(sv2v_tmp_fetcher_instance_mem_read_valid),
		.mem_read_address(sv2v_tmp_fetcher_instance_mem_read_address),
		.mem_read_ready(program_mem_read_ready),
		.mem_read_data(program_mem_read_data),
		.fetcher_state(sv2v_tmp_fetcher_instance_fetcher_state),
		.instruction(sv2v_tmp_fetcher_instance_instruction)
	);
	wire [4:1] sv2v_tmp_decoder_instance_decoded_rd_address;
	always @(*) decoded_rd_address = sv2v_tmp_decoder_instance_decoded_rd_address;
	wire [4:1] sv2v_tmp_decoder_instance_decoded_rs_address;
	always @(*) decoded_rs_address = sv2v_tmp_decoder_instance_decoded_rs_address;
	wire [4:1] sv2v_tmp_decoder_instance_decoded_rt_address;
	always @(*) decoded_rt_address = sv2v_tmp_decoder_instance_decoded_rt_address;
	wire [3:1] sv2v_tmp_decoder_instance_decoded_nzp;
	always @(*) decoded_nzp = sv2v_tmp_decoder_instance_decoded_nzp;
	wire [8:1] sv2v_tmp_decoder_instance_decoded_immediate;
	always @(*) decoded_immediate = sv2v_tmp_decoder_instance_decoded_immediate;
	wire [1:1] sv2v_tmp_decoder_instance_decoded_reg_write_enable;
	always @(*) decoded_reg_write_enable = sv2v_tmp_decoder_instance_decoded_reg_write_enable;
	wire [1:1] sv2v_tmp_decoder_instance_decoded_mem_read_enable;
	always @(*) decoded_mem_read_enable = sv2v_tmp_decoder_instance_decoded_mem_read_enable;
	wire [1:1] sv2v_tmp_decoder_instance_decoded_mem_write_enable;
	always @(*) decoded_mem_write_enable = sv2v_tmp_decoder_instance_decoded_mem_write_enable;
	wire [1:1] sv2v_tmp_decoder_instance_decoded_nzp_write_enable;
	always @(*) decoded_nzp_write_enable = sv2v_tmp_decoder_instance_decoded_nzp_write_enable;
	wire [2:1] sv2v_tmp_decoder_instance_decoded_reg_input_mux;
	always @(*) decoded_reg_input_mux = sv2v_tmp_decoder_instance_decoded_reg_input_mux;
	wire [2:1] sv2v_tmp_decoder_instance_decoded_alu_arithmetic_mux;
	always @(*) decoded_alu_arithmetic_mux = sv2v_tmp_decoder_instance_decoded_alu_arithmetic_mux;
	wire [1:1] sv2v_tmp_decoder_instance_decoded_alu_output_mux;
	always @(*) decoded_alu_output_mux = sv2v_tmp_decoder_instance_decoded_alu_output_mux;
	wire [1:1] sv2v_tmp_decoder_instance_decoded_pc_mux;
	always @(*) decoded_pc_mux = sv2v_tmp_decoder_instance_decoded_pc_mux;
	wire [1:1] sv2v_tmp_decoder_instance_decoded_ret;
	always @(*) decoded_ret = sv2v_tmp_decoder_instance_decoded_ret;
	decoder decoder_instance(
		.clk(clk),
		.reset(reset),
		.core_state(core_state),
		.instruction(instruction),
		.decoded_rd_address(sv2v_tmp_decoder_instance_decoded_rd_address),
		.decoded_rs_address(sv2v_tmp_decoder_instance_decoded_rs_address),
		.decoded_rt_address(sv2v_tmp_decoder_instance_decoded_rt_address),
		.decoded_nzp(sv2v_tmp_decoder_instance_decoded_nzp),
		.decoded_immediate(sv2v_tmp_decoder_instance_decoded_immediate),
		.decoded_reg_write_enable(sv2v_tmp_decoder_instance_decoded_reg_write_enable),
		.decoded_mem_read_enable(sv2v_tmp_decoder_instance_decoded_mem_read_enable),
		.decoded_mem_write_enable(sv2v_tmp_decoder_instance_decoded_mem_write_enable),
		.decoded_nzp_write_enable(sv2v_tmp_decoder_instance_decoded_nzp_write_enable),
		.decoded_reg_input_mux(sv2v_tmp_decoder_instance_decoded_reg_input_mux),
		.decoded_alu_arithmetic_mux(sv2v_tmp_decoder_instance_decoded_alu_arithmetic_mux),
		.decoded_alu_output_mux(sv2v_tmp_decoder_instance_decoded_alu_output_mux),
		.decoded_pc_mux(sv2v_tmp_decoder_instance_decoded_pc_mux),
		.decoded_ret(sv2v_tmp_decoder_instance_decoded_ret)
	);
	wire [3:1] sv2v_tmp_warp_scheduler_core_state;
	always @(*) core_state = sv2v_tmp_warp_scheduler_core_state;
	wire [MAX_WARPS_PER_CORE * 8:1] sv2v_tmp_warp_scheduler_warp_pc;
	always @(*) warp_pc = sv2v_tmp_warp_scheduler_warp_pc;
	wire [THREAD_ID_BITS:1] sv2v_tmp_warp_scheduler_current_warp_id;
	always @(*) current_warp_id = sv2v_tmp_warp_scheduler_current_warp_id;
	warps #(
		.THREADS_PER_WARP(THREADS_PER_WARP),
		.MAX_WARPS_PER_CORE(MAX_WARPS_PER_CORE),
		.THREAD_ID_BITS(THREAD_ID_BITS)
	) warp_scheduler(
		.clk(clk),
		.reset(reset),
		.start(start),
		.fetcher_state(fetcher_state),
		.core_state(sv2v_tmp_warp_scheduler_core_state),
		.decoded_mem_read_enable(decoded_mem_read_enable),
		.decoded_mem_write_enable(decoded_mem_write_enable),
		.decoded_ret(decoded_ret),
		.lsu_state(lsu_state),
		.thread_count(thread_count),
		.warp_pc(sv2v_tmp_warp_scheduler_warp_pc),
		.current_warp_id(sv2v_tmp_warp_scheduler_current_warp_id),
		.done(done)
	);
	genvar _gv_i_2;
	generate
		for (_gv_i_2 = 0; _gv_i_2 < THREADS_PER_WARP; _gv_i_2 = _gv_i_2 + 1) begin : threads
			localparam i = _gv_i_2;
			alu alu_instance(
				.clk(clk),
				.reset(reset),
				.core_state(core_state),
				.decoded_alu_arithmetic_mux(decoded_alu_arithmetic_mux),
				.decoded_alu_output_mux(decoded_alu_output_mux),
				.rs(rs[i]),
				.rt(rt[i]),
				.alu_out(alu_out[i])
			);
			wire [1:1] sv2v_tmp_lsu_instance_mem_read_valid;
			always @(*) data_mem_read_valid[i] = sv2v_tmp_lsu_instance_mem_read_valid;
			wire [DATA_MEM_ADDR_BITS * 1:1] sv2v_tmp_lsu_instance_mem_read_address;
			always @(*) data_mem_read_address[i * DATA_MEM_ADDR_BITS+:DATA_MEM_ADDR_BITS] = sv2v_tmp_lsu_instance_mem_read_address;
			wire [1:1] sv2v_tmp_lsu_instance_mem_write_valid;
			always @(*) data_mem_write_valid[i] = sv2v_tmp_lsu_instance_mem_write_valid;
			wire [DATA_MEM_ADDR_BITS * 1:1] sv2v_tmp_lsu_instance_mem_write_address;
			always @(*) data_mem_write_address[i * DATA_MEM_ADDR_BITS+:DATA_MEM_ADDR_BITS] = sv2v_tmp_lsu_instance_mem_write_address;
			wire [DATA_MEM_DATA_BITS * 1:1] sv2v_tmp_lsu_instance_mem_write_data;
			always @(*) data_mem_write_data[i * DATA_MEM_DATA_BITS+:DATA_MEM_DATA_BITS] = sv2v_tmp_lsu_instance_mem_write_data;
			wire [2:1] sv2v_tmp_lsu_instance_lsu_state;
			always @(*) lsu_state[i * 2+:2] = sv2v_tmp_lsu_instance_lsu_state;
			wire [8:1] sv2v_tmp_lsu_instance_lsu_out;
			always @(*) lsu_out[i] = sv2v_tmp_lsu_instance_lsu_out;
			lsu lsu_instance(
				.clk(clk),
				.reset(reset),
				.core_state(core_state),
				.decoded_mem_read_enable(decoded_mem_read_enable),
				.decoded_mem_write_enable(decoded_mem_write_enable),
				.mem_read_valid(sv2v_tmp_lsu_instance_mem_read_valid),
				.mem_read_address(sv2v_tmp_lsu_instance_mem_read_address),
				.mem_read_ready(data_mem_read_ready[i]),
				.mem_read_data(data_mem_read_data[i * DATA_MEM_DATA_BITS+:DATA_MEM_DATA_BITS]),
				.mem_write_valid(sv2v_tmp_lsu_instance_mem_write_valid),
				.mem_write_address(sv2v_tmp_lsu_instance_mem_write_address),
				.mem_write_data(sv2v_tmp_lsu_instance_mem_write_data),
				.mem_write_ready(data_mem_write_ready[i]),
				.rs(rs[i]),
				.rt(rt[i]),
				.lsu_state(sv2v_tmp_lsu_instance_lsu_state),
				.lsu_out(sv2v_tmp_lsu_instance_lsu_out)
			);
			wire [8:1] sv2v_tmp_register_instance_rd;
			always @(*) rd[i] = sv2v_tmp_register_instance_rd;
			wire [8:1] sv2v_tmp_register_instance_rs;
			always @(*) rs[i] = sv2v_tmp_register_instance_rs;
			wire [8:1] sv2v_tmp_register_instance_rt;
			always @(*) rt[i] = sv2v_tmp_register_instance_rt;
			registers #(
				.BLOCK_ID(CORE_ID),
				.THREAD_ID(i)
			) register_instance(
				.clk(clk),
				.reset(reset),
				.block_dim(block_dim),
				.core_state(core_state),
				.decoded_reg_write_enable(decoded_reg_write_enable),
				.decoded_reg_input_mux(decoded_reg_input_mux),
				.decoded_rd_address(decoded_rd_address),
				.decoded_rs_address(decoded_rs_address),
				.decoded_rt_address(decoded_rt_address),
				.decoded_immediate(decoded_immediate),
				.alu_out(alu_out[i]),
				.lsu_out(lsu_out[i]),
				.rd(sv2v_tmp_register_instance_rd),
				.rs(sv2v_tmp_register_instance_rs),
				.rt(sv2v_tmp_register_instance_rt)
			);
			wire [8:1] sv2v_tmp_pc_instance_next_pc;
			always @(*) next_pc[i] = sv2v_tmp_pc_instance_next_pc;
			pc pc_instance(
				.clk(clk),
				.reset(reset),
				.core_state(core_state),
				.decoded_nzp(decoded_nzp),
				.decoded_immediate(decoded_immediate),
				.decoded_nzp_write_enable(decoded_nzp_write_enable),
				.decoded_pc_mux(decoded_pc_mux),
				.alu_out(alu_out[i]),
				.current_pc(warp_pc[current_warp_id * 8+:8]),
				.next_pc(sv2v_tmp_pc_instance_next_pc)
			);
		end
	endgenerate
endmodule
`default_nettype none
module decoder (
	clk,
	reset,
	core_state,
	instruction,
	decoded_rd_address,
	decoded_rs_address,
	decoded_rt_address,
	decoded_nzp,
	decoded_immediate,
	decoded_reg_write_enable,
	decoded_mem_read_enable,
	decoded_mem_write_enable,
	decoded_nzp_write_enable,
	decoded_reg_input_mux,
	decoded_alu_arithmetic_mux,
	decoded_alu_output_mux,
	decoded_pc_mux,
	decoded_ret
);
	input wire clk;
	input wire reset;
	input wire [2:0] core_state;
	input wire [15:0] instruction;
	output reg [3:0] decoded_rd_address;
	output reg [3:0] decoded_rs_address;
	output reg [3:0] decoded_rt_address;
	output reg [2:0] decoded_nzp;
	output reg [7:0] decoded_immediate;
	output reg decoded_reg_write_enable;
	output reg decoded_mem_read_enable;
	output reg decoded_mem_write_enable;
	output reg decoded_nzp_write_enable;
	output reg [1:0] decoded_reg_input_mux;
	output reg [1:0] decoded_alu_arithmetic_mux;
	output reg decoded_alu_output_mux;
	output reg decoded_pc_mux;
	output reg decoded_ret;
	localparam NOP = 4'b0000;
	localparam BRnzp = 4'b0001;
	localparam CMP = 4'b0010;
	localparam ADD = 4'b0011;
	localparam SUB = 4'b0100;
	localparam MUL = 4'b0101;
	localparam DIV = 4'b0110;
	localparam LDR = 4'b0111;
	localparam STR = 4'b1000;
	localparam CONST = 4'b1001;
	localparam RET = 4'b1111;
	always @(posedge clk)
		if (reset) begin
			decoded_rd_address <= 0;
			decoded_rs_address <= 0;
			decoded_rt_address <= 0;
			decoded_immediate <= 0;
			decoded_nzp <= 0;
			decoded_reg_write_enable <= 0;
			decoded_mem_read_enable <= 0;
			decoded_mem_write_enable <= 0;
			decoded_nzp_write_enable <= 0;
			decoded_reg_input_mux <= 0;
			decoded_alu_arithmetic_mux <= 0;
			decoded_alu_output_mux <= 0;
			decoded_pc_mux <= 0;
			decoded_ret <= 0;
		end
		else if (core_state == 3'b010) begin
			decoded_rd_address <= instruction[11:8];
			decoded_rs_address <= instruction[7:4];
			decoded_rt_address <= instruction[3:0];
			decoded_immediate <= instruction[7:0];
			decoded_nzp <= instruction[11:9];
			decoded_reg_write_enable <= 0;
			decoded_mem_read_enable <= 0;
			decoded_mem_write_enable <= 0;
			decoded_nzp_write_enable <= 0;
			decoded_reg_input_mux <= 0;
			decoded_alu_arithmetic_mux <= 0;
			decoded_alu_output_mux <= 0;
			decoded_pc_mux <= 0;
			decoded_ret <= 0;
			case (instruction[15:12])
				NOP:
					;
				BRnzp: decoded_pc_mux <= 1;
				CMP: begin
					decoded_alu_output_mux <= 1;
					decoded_nzp_write_enable <= 1;
				end
				ADD: begin
					decoded_reg_write_enable <= 1;
					decoded_reg_input_mux <= 2'b00;
					decoded_alu_arithmetic_mux <= 2'b00;
				end
				SUB: begin
					decoded_reg_write_enable <= 1;
					decoded_reg_input_mux <= 2'b00;
					decoded_alu_arithmetic_mux <= 2'b01;
				end
				MUL: begin
					decoded_reg_write_enable <= 1;
					decoded_reg_input_mux <= 2'b00;
					decoded_alu_arithmetic_mux <= 2'b10;
				end
				DIV: begin
					decoded_reg_write_enable <= 1;
					decoded_reg_input_mux <= 2'b00;
					decoded_alu_arithmetic_mux <= 2'b11;
				end
				LDR: begin
					decoded_reg_write_enable <= 1;
					decoded_reg_input_mux <= 2'b01;
					decoded_mem_read_enable <= 1;
				end
				STR: decoded_mem_write_enable <= 1;
				CONST: begin
					decoded_reg_write_enable <= 1;
					decoded_reg_input_mux <= 2'b10;
				end
				RET: decoded_ret <= 1;
			endcase
		end
endmodule
`default_nettype none
module fetcher (
	clk,
	reset,
	core_state,
	current_pc,
	mem_read_valid,
	mem_read_address,
	mem_read_ready,
	mem_read_data,
	fetcher_state,
	instruction
);
	parameter PROGRAM_MEM_ADDR_BITS = 8;
	parameter PROGRAM_MEM_DATA_BITS = 16;
	input wire clk;
	input wire reset;
	input wire [2:0] core_state;
	input wire [7:0] current_pc;
	output reg mem_read_valid;
	output reg [PROGRAM_MEM_ADDR_BITS - 1:0] mem_read_address;
	input wire mem_read_ready;
	input wire [PROGRAM_MEM_DATA_BITS - 1:0] mem_read_data;
	output reg [2:0] fetcher_state;
	output reg [PROGRAM_MEM_DATA_BITS - 1:0] instruction;
	localparam IDLE = 3'b000;
	localparam FETCHING = 3'b001;
	localparam FETCHED = 3'b010;
	always @(posedge clk)
		if (reset) begin
			fetcher_state <= IDLE;
			mem_read_valid <= 0;
			instruction <= {PROGRAM_MEM_DATA_BITS {1'b0}};
		end
		else
			case (fetcher_state)
				IDLE:
					if (core_state == 3'b001) begin
						fetcher_state <= FETCHING;
						mem_read_valid <= 1;
						mem_read_address <= current_pc;
					end
				FETCHING:
					if (mem_read_ready) begin
						fetcher_state <= FETCHED;
						instruction <= mem_read_data;
						mem_read_valid <= 0;
					end
				FETCHED:
					if (core_state == 3'b010)
						fetcher_state <= IDLE;
			endcase
endmodule
`default_nettype none
module gpu (
	clk,
	reset,
	start,
	done,
	device_control_write_enable,
	device_control_data,
	program_mem_read_valid,
	program_mem_read_address,
	program_mem_read_ready,
	program_mem_read_data,
	data_mem_read_valid,
	data_mem_read_address,
	data_mem_read_ready,
	data_mem_read_data,
	data_mem_write_valid,
	data_mem_write_address,
	data_mem_write_data,
	data_mem_write_ready
);
	parameter DATA_MEM_ADDR_BITS = 8;
	parameter DATA_MEM_DATA_BITS = 8;
	parameter PROGRAM_MEM_ADDR_BITS = 8;
	parameter PROGRAM_MEM_DATA_BITS = 16;
	parameter NUM_CORES = 1;
	parameter MAX_WARPS_PER_CORE = 2;
	parameter THREADS_PER_WARP = 2;
	input wire clk;
	input wire reset;
	input wire start;
	output wire done;
	input wire device_control_write_enable;
	input wire [7:0] device_control_data;
	output wire program_mem_read_valid;
	output wire [PROGRAM_MEM_ADDR_BITS - 1:0] program_mem_read_address;
	input wire program_mem_read_ready;
	input wire [PROGRAM_MEM_DATA_BITS - 1:0] program_mem_read_data;
	output wire data_mem_read_valid;
	output wire [DATA_MEM_ADDR_BITS - 1:0] data_mem_read_address;
	input wire data_mem_read_ready;
	input wire [DATA_MEM_DATA_BITS - 1:0] data_mem_read_data;
	output wire data_mem_write_valid;
	output wire [DATA_MEM_ADDR_BITS - 1:0] data_mem_write_address;
	output wire [DATA_MEM_DATA_BITS - 1:0] data_mem_write_data;
	input wire data_mem_write_ready;
	reg [7:0] device_conrol_register;
	wire [7:0] thread_count;
	wire [7:0] block_dim;
	wire [7:0] block_thread_count [NUM_CORES - 1:0];
	wire [NUM_CORES - 1:0] core_done;
	assign thread_count = device_conrol_register[7:0];
	assign block_dim = ((thread_count + NUM_CORES) - 1) / NUM_CORES;
	genvar _gv_j_1;
	generate
		for (_gv_j_1 = 0; _gv_j_1 < NUM_CORES; _gv_j_1 = _gv_j_1 + 1) begin : block_thread_count_assignment
			localparam j = _gv_j_1;
			assign block_thread_count[j] = (j == (NUM_CORES - 1) ? thread_count - (block_dim * j) : block_dim);
		end
	endgenerate
	localparam NUM_LSUS = NUM_CORES * THREADS_PER_WARP;
	wire [NUM_LSUS - 1:0] lsu_read_valid;
	wire [(NUM_LSUS * DATA_MEM_ADDR_BITS) - 1:0] lsu_read_address;
	wire [NUM_LSUS - 1:0] lsu_read_ready;
	wire [(NUM_LSUS * DATA_MEM_DATA_BITS) - 1:0] lsu_read_data;
	wire [NUM_LSUS - 1:0] lsu_write_valid;
	wire [(NUM_LSUS * DATA_MEM_ADDR_BITS) - 1:0] lsu_write_address;
	wire [(NUM_LSUS * DATA_MEM_DATA_BITS) - 1:0] lsu_write_data;
	wire [NUM_LSUS - 1:0] lsu_write_ready;
	localparam NUM_FETCHERS = NUM_CORES;
	wire [NUM_FETCHERS - 1:0] fetcher_read_valid;
	wire [(NUM_FETCHERS * PROGRAM_MEM_ADDR_BITS) - 1:0] fetcher_read_address;
	wire [NUM_FETCHERS - 1:0] fetcher_read_ready;
	wire [(NUM_FETCHERS * PROGRAM_MEM_DATA_BITS) - 1:0] fetcher_read_data;
	controller #(
		.ADDR_BITS(DATA_MEM_ADDR_BITS),
		.DATA_BITS(DATA_MEM_DATA_BITS),
		.NUM_CONSUMERS(NUM_LSUS)
	) data_memory_controller(
		.clk(clk),
		.reset(reset),
		.consumer_read_valid(lsu_read_valid),
		.consumer_read_address(lsu_read_address),
		.consumer_read_ready(lsu_read_ready),
		.consumer_read_data(lsu_read_data),
		.consumer_write_valid(lsu_write_valid),
		.consumer_write_address(lsu_write_address),
		.consumer_write_data(lsu_write_data),
		.consumer_write_ready(lsu_write_ready),
		.mem_read_valid(data_mem_read_valid),
		.mem_read_address(data_mem_read_address),
		.mem_read_ready(data_mem_read_ready),
		.mem_read_data(data_mem_read_data),
		.mem_write_valid(data_mem_write_valid),
		.mem_write_address(data_mem_write_address),
		.mem_write_data(data_mem_write_data),
		.mem_write_ready(data_mem_write_ready)
	);
	controller #(
		.ADDR_BITS(PROGRAM_MEM_ADDR_BITS),
		.DATA_BITS(PROGRAM_MEM_DATA_BITS),
		.NUM_CONSUMERS(NUM_FETCHERS)
	) program_memory_controller(
		.clk(clk),
		.reset(reset),
		.consumer_read_valid(fetcher_read_valid),
		.consumer_read_address(fetcher_read_address),
		.consumer_read_ready(fetcher_read_ready),
		.consumer_read_data(fetcher_read_data),
		.mem_read_valid(program_mem_read_valid),
		.mem_read_address(program_mem_read_address),
		.mem_read_ready(program_mem_read_ready),
		.mem_read_data(program_mem_read_data)
	);
	genvar _gv_i_3;
	generate
		for (_gv_i_3 = 0; _gv_i_3 < NUM_CORES; _gv_i_3 = _gv_i_3 + 1) begin : cores
			localparam i = _gv_i_3;
			reg [THREADS_PER_WARP - 1:0] core_lsu_read_valid;
			reg [(THREADS_PER_WARP * DATA_MEM_ADDR_BITS) - 1:0] core_lsu_read_address;
			reg [THREADS_PER_WARP - 1:0] core_lsu_read_ready;
			reg [(THREADS_PER_WARP * DATA_MEM_DATA_BITS) - 1:0] core_lsu_read_data;
			reg [THREADS_PER_WARP - 1:0] core_lsu_write_valid;
			reg [(THREADS_PER_WARP * DATA_MEM_ADDR_BITS) - 1:0] core_lsu_write_address;
			reg [(THREADS_PER_WARP * DATA_MEM_DATA_BITS) - 1:0] core_lsu_write_data;
			reg [THREADS_PER_WARP - 1:0] core_lsu_write_ready;
			genvar _gv_j_2;
			for (_gv_j_2 = 0; _gv_j_2 < THREADS_PER_WARP; _gv_j_2 = _gv_j_2 + 1) begin : genblk1
				localparam j = _gv_j_2;
				localparam lsu_index = (i * THREADS_PER_WARP) + j;
				assign lsu_read_valid[lsu_index] = core_lsu_read_valid[j];
				assign lsu_read_address[lsu_index * DATA_MEM_ADDR_BITS+:DATA_MEM_ADDR_BITS] = core_lsu_read_address[j * DATA_MEM_ADDR_BITS+:DATA_MEM_ADDR_BITS];
				assign lsu_write_valid[lsu_index] = core_lsu_write_valid[j];
				assign lsu_write_address[lsu_index * DATA_MEM_ADDR_BITS+:DATA_MEM_ADDR_BITS] = core_lsu_write_address[j * DATA_MEM_ADDR_BITS+:DATA_MEM_ADDR_BITS];
				assign lsu_write_data[lsu_index * DATA_MEM_DATA_BITS+:DATA_MEM_DATA_BITS] = core_lsu_write_data[j * DATA_MEM_DATA_BITS+:DATA_MEM_DATA_BITS];
				always @(posedge clk) begin
					core_lsu_read_ready[j] <= lsu_read_ready[lsu_index];
					core_lsu_read_data[j * DATA_MEM_DATA_BITS+:DATA_MEM_DATA_BITS] <= lsu_read_data[lsu_index * DATA_MEM_DATA_BITS+:DATA_MEM_DATA_BITS];
					core_lsu_write_ready[j] <= lsu_write_ready[lsu_index];
				end
			end
			wire [THREADS_PER_WARP:1] sv2v_tmp_core_instance_data_mem_read_valid;
			always @(*) core_lsu_read_valid = sv2v_tmp_core_instance_data_mem_read_valid;
			wire [THREADS_PER_WARP * DATA_MEM_ADDR_BITS:1] sv2v_tmp_core_instance_data_mem_read_address;
			always @(*) core_lsu_read_address = sv2v_tmp_core_instance_data_mem_read_address;
			wire [THREADS_PER_WARP:1] sv2v_tmp_core_instance_data_mem_write_valid;
			always @(*) core_lsu_write_valid = sv2v_tmp_core_instance_data_mem_write_valid;
			wire [THREADS_PER_WARP * DATA_MEM_ADDR_BITS:1] sv2v_tmp_core_instance_data_mem_write_address;
			always @(*) core_lsu_write_address = sv2v_tmp_core_instance_data_mem_write_address;
			wire [THREADS_PER_WARP * DATA_MEM_DATA_BITS:1] sv2v_tmp_core_instance_data_mem_write_data;
			always @(*) core_lsu_write_data = sv2v_tmp_core_instance_data_mem_write_data;
			core #(
				.DATA_MEM_ADDR_BITS(DATA_MEM_ADDR_BITS),
				.DATA_MEM_DATA_BITS(DATA_MEM_DATA_BITS),
				.PROGRAM_MEM_ADDR_BITS(PROGRAM_MEM_ADDR_BITS),
				.PROGRAM_MEM_DATA_BITS(PROGRAM_MEM_DATA_BITS),
				.MAX_WARPS_PER_CORE(MAX_WARPS_PER_CORE),
				.THREADS_PER_WARP(THREADS_PER_WARP),
				.CORE_ID(i)
			) core_instance(
				.clk(clk),
				.reset(reset),
				.start(start),
				.done(core_done[i]),
				.block_dim(block_dim),
				.thread_count(block_thread_count[i]),
				.program_mem_read_valid(fetcher_read_valid[i]),
				.program_mem_read_address(fetcher_read_address[i * PROGRAM_MEM_ADDR_BITS+:PROGRAM_MEM_ADDR_BITS]),
				.program_mem_read_ready(fetcher_read_ready[i]),
				.program_mem_read_data(fetcher_read_data[i * PROGRAM_MEM_DATA_BITS+:PROGRAM_MEM_DATA_BITS]),
				.data_mem_read_valid(sv2v_tmp_core_instance_data_mem_read_valid),
				.data_mem_read_address(sv2v_tmp_core_instance_data_mem_read_address),
				.data_mem_read_ready(core_lsu_read_ready),
				.data_mem_read_data(core_lsu_read_data),
				.data_mem_write_valid(sv2v_tmp_core_instance_data_mem_write_valid),
				.data_mem_write_address(sv2v_tmp_core_instance_data_mem_write_address),
				.data_mem_write_data(sv2v_tmp_core_instance_data_mem_write_data),
				.data_mem_write_ready(core_lsu_write_ready)
			);
		end
	endgenerate
	always @(posedge clk)
		if (reset)
			device_conrol_register <= 8'b00000000;
		else if (device_control_write_enable)
			device_conrol_register <= device_control_data;
	assign done = &core_done;
endmodule
`default_nettype none
module lsu (
	clk,
	reset,
	core_state,
	decoded_mem_read_enable,
	decoded_mem_write_enable,
	mem_read_valid,
	mem_read_address,
	mem_read_ready,
	mem_read_data,
	mem_write_valid,
	mem_write_address,
	mem_write_data,
	mem_write_ready,
	rs,
	rt,
	lsu_state,
	lsu_out
);
	input wire clk;
	input wire reset;
	input wire [2:0] core_state;
	input wire decoded_mem_read_enable;
	input wire decoded_mem_write_enable;
	output reg mem_read_valid;
	output reg [7:0] mem_read_address;
	input wire mem_read_ready;
	input wire [7:0] mem_read_data;
	output reg mem_write_valid;
	output reg [7:0] mem_write_address;
	output reg [7:0] mem_write_data;
	input wire mem_write_ready;
	input wire [7:0] rs;
	input wire [7:0] rt;
	output reg [1:0] lsu_state;
	output reg [7:0] lsu_out;
	localparam IDLE = 2'b00;
	localparam WAITING = 2'b01;
	localparam DONE = 2'b10;
	always @(posedge clk)
		if (reset)
			lsu_state <= IDLE;
		else begin
			if (decoded_mem_read_enable)
				case (lsu_state)
					IDLE:
						if (core_state == 3'b011) begin
							mem_read_valid <= 1;
							mem_read_address <= rs;
							lsu_state <= WAITING;
						end
					WAITING:
						if (mem_read_ready == 1) begin
							mem_read_valid <= 0;
							lsu_out <= mem_read_data;
							lsu_state <= DONE;
						end
					DONE:
						if (core_state == 3'b101)
							lsu_state <= IDLE;
				endcase
			if (decoded_mem_write_enable)
				case (lsu_state)
					IDLE:
						if (core_state == 3'b011) begin
							mem_write_valid <= 1;
							mem_write_address <= rs;
							mem_write_data <= rt;
							lsu_state <= WAITING;
						end
					WAITING:
						if (mem_write_ready) begin
							mem_write_valid <= 0;
							lsu_state <= DONE;
						end
					DONE:
						if (core_state == 3'b101)
							lsu_state <= IDLE;
				endcase
		end
endmodule
`default_nettype none
module pc (
	clk,
	reset,
	core_state,
	decoded_nzp,
	decoded_immediate,
	decoded_nzp_write_enable,
	decoded_pc_mux,
	alu_out,
	current_pc,
	next_pc
);
	parameter DATA_MEM_DATA_BITS = 8;
	parameter PROGRAM_MEM_ADDR_BITS = 8;
	input wire clk;
	input wire reset;
	input wire [2:0] core_state;
	input wire [2:0] decoded_nzp;
	input wire [DATA_MEM_DATA_BITS - 1:0] decoded_immediate;
	input wire decoded_nzp_write_enable;
	input wire decoded_pc_mux;
	input wire [DATA_MEM_DATA_BITS - 1:0] alu_out;
	input wire [PROGRAM_MEM_ADDR_BITS - 1:0] current_pc;
	output reg [PROGRAM_MEM_ADDR_BITS - 1:0] next_pc;
	reg [2:0] nzp;
	always @(posedge clk)
		if (reset)
			nzp <= 3'b000;
		else if (core_state == 3'b101) begin
			if (decoded_nzp_write_enable)
				nzp <= alu_out[2:0];
			if (decoded_pc_mux == 1) begin
				if ((nzp & decoded_nzp) != 3'b000)
					next_pc <= decoded_immediate;
			end
			else
				next_pc <= current_pc + 1;
		end
endmodule
`default_nettype none
module registers (
	clk,
	reset,
	block_dim,
	core_state,
	decoded_reg_write_enable,
	decoded_reg_input_mux,
	decoded_rd_address,
	decoded_rs_address,
	decoded_rt_address,
	decoded_immediate,
	alu_out,
	lsu_out,
	rd,
	rs,
	rt
);
	parameter BLOCK_ID = 0;
	parameter THREAD_ID = 0;
	parameter DATA_BITS = 8;
	input wire clk;
	input wire reset;
	input wire [7:0] block_dim;
	input wire [2:0] core_state;
	input wire decoded_reg_write_enable;
	input wire [1:0] decoded_reg_input_mux;
	input wire [3:0] decoded_rd_address;
	input wire [3:0] decoded_rs_address;
	input wire [3:0] decoded_rt_address;
	input wire [DATA_BITS - 1:0] decoded_immediate;
	input wire [DATA_BITS - 1:0] alu_out;
	input wire [DATA_BITS - 1:0] lsu_out;
	output reg [7:0] rd;
	output reg [7:0] rs;
	output reg [7:0] rt;
	localparam ARITHMETIC = 2'b00;
	localparam MEMORY = 2'b01;
	localparam CONSTANT = 2'b10;
	reg [7:0] registers [15:0];
	always @(posedge clk)
		if (reset) begin
			registers[0] <= 8'b00000000;
			registers[1] <= 8'b00000000;
			registers[2] <= 8'b00000000;
			registers[3] <= 8'b00000000;
			registers[4] <= 8'b00000000;
			registers[5] <= 8'b00000000;
			registers[6] <= 8'b00000000;
			registers[7] <= 8'b00000000;
			registers[8] <= 8'b00000000;
			registers[9] <= 8'b00000000;
			registers[10] <= 8'b00000000;
			registers[11] <= 8'b00000000;
			registers[12] <= 8'b00000000;
			registers[13] <= BLOCK_ID;
			registers[14] <= block_dim;
			registers[15] <= THREAD_ID;
		end
		else begin
			if (core_state == 3'b011) begin
				rs <= registers[decoded_rs_address];
				rt <= registers[decoded_rt_address];
			end
			if (core_state == 3'b100)
				case (decoded_reg_input_mux)
					ARITHMETIC: rd <= alu_out;
					MEMORY: rd <= lsu_out;
					CONSTANT: rd <= decoded_immediate;
				endcase
			if (core_state == 3'b101) begin
				if (decoded_reg_write_enable && (decoded_rd_address < 13))
					registers[decoded_rd_address] <= rd;
			end
			registers[14] <= block_dim;
		end
endmodule
`default_nettype none
module warps (
	clk,
	reset,
	start,
	fetcher_state,
	core_state,
	decoded_mem_read_enable,
	decoded_mem_write_enable,
	decoded_ret,
	lsu_state,
	thread_count,
	warp_pc,
	current_warp_id,
	done
);
	parameter MAX_WARPS_PER_CORE = 2;
	parameter THREADS_PER_WARP = 2;
	parameter THREAD_ID_BITS = $clog2(THREADS_PER_WARP);
	input wire clk;
	input wire reset;
	input wire start;
	input wire [2:0] fetcher_state;
	output reg [2:0] core_state;
	input wire decoded_mem_read_enable;
	input wire decoded_mem_write_enable;
	input wire decoded_ret;
	input wire [(THREADS_PER_WARP * 2) - 1:0] lsu_state;
	input wire [7:0] thread_count;
	output reg [(THREADS_PER_WARP * 8) - 1:0] warp_pc;
	output reg [THREAD_ID_BITS - 1:0] current_warp_id;
	output wire done;
	localparam IDLE = 3'b000;
	localparam FETCH = 3'b001;
	localparam DECODE = 3'b010;
	localparam REQUEST = 3'b011;
	localparam EXECUTE = 3'b100;
	localparam UPDATE = 3'b101;
	wire [7:0] NUM_WARPS = ((thread_count + THREADS_PER_WARP) - 1) / THREADS_PER_WARP;
	reg [MAX_WARPS_PER_CORE - 1:0] warp_done;
	assign done = &warp_done;
	always @(posedge clk)
		if (reset) begin
			current_warp_id <= 0;
			core_state <= IDLE;
			begin : sv2v_autoblock_1
				reg signed [31:0] i;
				for (i = 0; i < MAX_WARPS_PER_CORE; i = i + 1)
					begin
						warp_pc[i * 8+:8] <= 0;
						warp_done[i] <= 0;
					end
			end
		end
		else
			case (core_state)
				IDLE:
					if (start)
						core_state <= FETCH;
				FETCH:
					if (warp_done[current_warp_id])
						current_warp_id <= (current_warp_id + 1) % NUM_WARPS;
					else if (fetcher_state == 3'b010)
						core_state <= DECODE;
				DECODE: core_state <= REQUEST;
				REQUEST:
					if (decoded_mem_read_enable || decoded_mem_write_enable)
						core_state <= EXECUTE;
					else
						core_state <= EXECUTE;
				EXECUTE: core_state <= UPDATE;
				UPDATE: begin
					if (decoded_ret)
						warp_done[current_warp_id] <= 1;
					core_state <= FETCH;
					current_warp_id <= (current_warp_id + 1) % NUM_WARPS;
				end
			endcase
endmodule
`default_nettype none
module alu (
	clk,
	reset,
	core_state,
	decoded_alu_arithmetic_mux,
	decoded_alu_output_mux,
	rs,
	rt,
	alu_out
);
	input wire clk;
	input wire reset;
	input wire [2:0] core_state;
	input wire [1:0] decoded_alu_arithmetic_mux;
	input wire decoded_alu_output_mux;
	input wire [7:0] rs;
	input wire [7:0] rt;
	output reg [7:0] alu_out;
	localparam ADD = 2'b00;
	localparam SUB = 2'b01;
	localparam MUL = 2'b10;
	localparam DIV = 2'b11;
	always @(posedge clk)
		if (reset)
			alu_out <= 8'b00000000;
		else begin
			alu_out <= 8'b00000000;
			if (core_state == 3'b100) begin
				if (decoded_alu_output_mux == 1)
					alu_out <= {5'b00000, (rs - rt) > 0, (rs - rt) == 0, (rs - rt) < 0};
				else
					case (decoded_alu_arithmetic_mux)
						ADD: alu_out <= rs + rt;
						SUB: alu_out <= rs - rt;
						MUL: alu_out <= rs * rt;
						DIV: alu_out <= rs / rt;
					endcase
			end
		end
endmodule