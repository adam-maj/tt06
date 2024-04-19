/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_adammaj (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
  wire device_control_write_enable;
  wire [7:0] device_control_data;
  wire program_mem_read_valid;
  wire [7:0] program_mem_read_address;
  reg program_mem_read_ready;
  reg [15:0] program_mem_read_data;

  wire data_mem_read_valid;
  wire [7:0] data_mem_read_address;
  reg data_mem_read_ready;
  reg [7:0] data_mem_read_data;
  wire data_mem_write_valid;
  wire [7:0] data_mem_write_address;
  wire [7:0] data_mem_write_data;
  reg data_mem_write_ready;

  gpu tt_gpu (
    .clk(clk),
    .reset(~rst_n),
    .device_control_write_enable(device_control_write_enable),
    .device_control_data(device_control_data),
    .program_mem_read_valid(program_mem_read_valid),
    .program_mem_read_address(program_mem_read_address),
    .program_mem_read_ready(program_mem_read_ready),
    .program_mem_read_data(program_mem_read_data),
    .data_mem_read_valid(data_mem_read_valid),
    .data_mem_read_address(data_mem_read_address),
    .data_mem_read_data(data_mem_read_data),
    .data_mem_write_valid(data_mem_write_valid),
    .data_mem_write_address(data_mem_write_address),
    .data_mem_write_data(data_mem_write_data),
    .data_mem_write_ready(data_mem_write_ready)
  );

  // All output pins must be assigned. If not used, assign to 0.
  assign uo_out  = ui_in + uio_in;  // Example: ou_out is the sum of ui_in and uio_in
  assign uio_out = 0;
  assign uio_oe  = 0;
endmodule
