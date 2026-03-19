// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Adapted from picobello for MX RedMule integration.
// Removed DataMover and HCI mux — only MX RedMule remains.

`include "hci_helpers.svh"

module snitch_hwpe_subsystem
  import hci_package::*;
  import hwpe_ctrl_package::*;
  import reqrsp_pkg::amo_op_e;
#(
  parameter type         tcdm_req_t    = logic,
  parameter type         tcdm_rsp_t    = logic,
  parameter type         periph_req_t  = logic,
  parameter type         periph_rsp_t  = logic,
  parameter int unsigned HwpeDataWidth = 512,
  parameter int unsigned IdWidth       = 8,
  parameter int unsigned NrCores       = 8,
  parameter int unsigned TCDMDataWidth = 64
) (
  input logic clk_i,
  input logic rst_ni,
  input logic test_mode_i,

  // TCDM interface (Master) — RedMule data path
  output tcdm_req_t tcdm_req_o,
  input  tcdm_rsp_t tcdm_rsp_i,

  // HWPE control interface (Slave)
  input  periph_req_t hwpe_ctrl_req_i,
  output periph_rsp_t hwpe_ctrl_rsp_o,

  // Core event interrupts
  output logic [NrCores-1:0] hwpe_evt_o,

  // FIXME: mx_exp_stream is currently drained and discarded (ready=1).
  // Route exponent writes through RedMule's streamer/TCDM instead,
  // so output MX exponents are written to memory alongside data.
  output logic mx_exp_stream_busy_o
);

  // verilog_format: off
  localparam hci_size_parameter_t HCISizeTcdm = '{
    DW:  HwpeDataWidth,
    AW:  DEFAULT_AW,
    BW:  DEFAULT_BW,
    UW:  DEFAULT_UW,
    IW:  DEFAULT_IW,
    EW:  0,
    EHW: 0
  };
  // verilog_format: on

  logic                         hwpe_clk;
  logic                         clk_en;

  logic [NrCores-1:0][1:0]      evt;
  logic                         busy;

  // Machine HWPE Interrupt
  logic [NrCores-1:0] hwpe_evt_d, hwpe_evt_q;

  hwpe_ctrl_intf_periph #(.ID_WIDTH(IdWidth)) periph (.clk(clk_i));

  hci_core_intf #(
`ifndef SYNTHESIS
    .WAIVE_RSP3_ASSERT(1'b1),
`endif
    .DW               (HwpeDataWidth),
    .EW               (0),
    .EHW              (0)
  ) tcdm (
    .clk(clk_i)
  );

  // FIXME: mx_exp_stream drained here — exponent data is lost.
  // Fix by routing through RedMule's streamer to TCDM.
  hwpe_stream_intf_stream #(.DATA_WIDTH(32)) mx_exp_stream (.clk(clk_i));
  assign mx_exp_stream.ready = 1'b1;
  assign mx_exp_stream_busy_o = mx_exp_stream.valid;

  // Dummy XIF interface (unused — we use HWPE periph mode, X_EXT=0)
  cv32e40x_if_xif xif_dummy ();

  // ----- TCDM request/response mapping -----

  // request channel
  assign tcdm_req_o.q_valid = tcdm.req;
  assign tcdm_req_o.q.addr  = tcdm.add;
  assign tcdm_req_o.q.write = ~tcdm.wen;
  assign tcdm_req_o.q.strb  = tcdm.be;
  assign tcdm_req_o.q.data  = tcdm.data;
  assign tcdm_req_o.q.amo   = reqrsp_pkg::AMONone;
  assign tcdm_req_o.q.user  = '0;
  // response channel
  assign tcdm.gnt           = tcdm_rsp_i.q_ready;
  assign tcdm.r_valid       = tcdm_rsp_i.p_valid;
  assign tcdm.r_data        = tcdm_rsp_i.p.data;
  assign tcdm.r_opc         = '0;
  assign tcdm.r_user        = '0;

  // ----- Periph routing (single target: RedMule) -----

  always_comb begin
    periph.req           = '0;
    hwpe_ctrl_rsp_o.q_ready = '0;
    hwpe_ctrl_rsp_o.p.data  = '0;
    hwpe_ctrl_rsp_o.p_valid = '0;

    periph.add           = {24'h0, hwpe_ctrl_req_i.q.addr[7:0]};
    periph.wen           = ~hwpe_ctrl_req_i.q.write;
    periph.be            = hwpe_ctrl_req_i.q.strb;
    periph.data          = hwpe_ctrl_req_i.q.data;
    periph.id            = hwpe_ctrl_req_i.q.user;

    // Clock enable register at 0x9C
    if (hwpe_ctrl_req_i.q.addr[7:0] == 'h9C) begin
      hwpe_ctrl_rsp_o.q_ready = hwpe_ctrl_req_i.q_valid;
      hwpe_ctrl_rsp_o.p_valid = '1;
    end else begin
      periph.req              = hwpe_ctrl_req_i.q_valid;
      hwpe_ctrl_rsp_o.q_ready = periph.gnt;
      hwpe_ctrl_rsp_o.p.data  = periph.r_data;
      hwpe_ctrl_rsp_o.p_valid = periph.r_valid;
    end
  end

  // ----- Clock enable register -----

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      clk_en <= '0;
    end else begin
      if (hwpe_ctrl_req_i.q.addr[7:0] == 'h9C && hwpe_ctrl_req_i.q_valid &&
          hwpe_ctrl_req_i.q.write) begin
        clk_en <= hwpe_ctrl_req_i.q.data[0];
      end
    end
  end

  // ----- Event handling -----

  for (genvar ii = 0; ii < NrCores; ii++) begin : gen_hwpe_evt
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (~rst_ni) begin
        hwpe_evt_q[ii] <= '0;
      end else begin
        if (evt[ii]) begin
          hwpe_evt_q[ii] <= 1'b1;
        end
        else if (hwpe_ctrl_req_i.q.addr[7:0] == 'h94 && hwpe_ctrl_req_i.q_valid &&
                 hwpe_ctrl_req_i.q.write && hwpe_ctrl_req_i.q.data == (1 << ii)) begin
          hwpe_evt_q[ii] <= 1'b0;
        end
      end
    end
  end
  assign hwpe_evt_o = hwpe_evt_q;

  // ----- Clock gating -----

  tc_clk_gating i_redmule_clk_gate (
    .clk_i    (clk_i),
    .en_i     (clk_en),
    .test_en_i('0),
    .clk_o    (hwpe_clk)
  );

  // ----- MX RedMule instantiation -----

  redmule_top #(
    .ID_WIDTH      (IdWidth),
    .N_CORES       (NrCores),
    .DW            (HwpeDataWidth),
    .X_EXT         (0),
    .HCI_SIZE_tcdm (HCISizeTcdm)
  ) i_redmule_top (
    .clk_i              (hwpe_clk),
    .rst_ni             (rst_ni),
    .test_mode_i        (test_mode_i),
    .evt_o              (evt),
    .busy_o             (busy),
    // XIF unused (X_EXT=0), but ports must be connected
    .xif_issue_if_i     (xif_dummy),
    .xif_result_if_o    (xif_dummy),
    .xif_compressed_if_i(xif_dummy),
    .xif_mem_if_o       (xif_dummy),
    // HWPE periph control
    .periph             (periph),
    // TCDM data path
    .tcdm               (tcdm),
    // MX exponent stream
    .mx_exp_stream      (mx_exp_stream)
  );

endmodule : snitch_hwpe_subsystem
