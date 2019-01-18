onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -format Logic -radix hexadecimal /tb_top/clk
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/iimx_adr_o
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/iimx_rqt_o
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/iimx_rty_i
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/iimx_dat_i
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/iimx_ack_i
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/iimx_err_i
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/iimx_adr_i
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/pc
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/npc_value
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/npc_branch
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/npc_branch_valid
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/npc_except
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_if/npc_except_valid
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/pc_freeze
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/if_freeze
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/id_freeze
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/flushpipe
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_pipectrl/flush_branch
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_pipectrl/flush_except
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_pipectrl/multicycle_stall
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/if_stall
add wave -noupdate -format Logic -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/if_rqt_valid
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/id_inst
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_id/ex_inst
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/id_cia
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_if/id_snia
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs0
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs1
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs2
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs3
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs4
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs5
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs6
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs7
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs8
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs9
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs10
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs11
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs12
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs13
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs14
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs15
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs16
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs17
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs18
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs19
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs20
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs21
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs22
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs23
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs24
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs25
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs26
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs27
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs28
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs29
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs30
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/reg_gprs/reg_rf/gprs31
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_bpu/lr
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_bpu/ctr
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_sprs/cr
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_sprs/msr
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_sprs/xer
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_sprs/dsurx
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_sprs/dsutx
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_sprs/dsuctrl
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_sprs/dsusta
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsurx_we
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsutx_we
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsuctrl_we
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsusta_we
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_sprs/spr_dat_wr_o
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_sprs/reg_uops
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_sprs/reg_addr
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_sprs/dat_i
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_sprs/spr_wb_dat
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/write_spr
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsuctrl_sel
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsuctrl_we
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsurx_sel
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsurx_we
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsusta_sel
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsusta_we
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsutx_sel
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_sprs/dsutx_we
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/flag_except
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/extend_flush
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_except/npc_except
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/npc_exp_valid
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_except/except_trig
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_except/except_type
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_except/state
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_except/state_next
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/rfci_go
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/rfi_go
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_align
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_isync
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_dbuserr
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_eieio
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_ext_ci
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_ext_i
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_fit
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_ibuserr
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_illegal
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_isync
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_pit
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_prg
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_rfci
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_rfi
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_svm_check
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_sync
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_syscall
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_trap
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_emulate
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_except/sig_watchdog
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_lsu/lsu_uops
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_lsu/lsu_addr
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_lsu/lsu_stall
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_lsu/lsu_done
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_lsu/dimx_rqt_o
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_lsu/dimx_adr_o
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_lsu/dimx_dat_o
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_lsu/dimx_ack_i
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_lsu/dimx_dat_i
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_lsu/dimx_err_i
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_lsu/dimx_sel_o
add wave -noupdate -format Logic /tb_top/top_pss/pippo_core/pippo_lsu/dimx_we_o
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_lsu/lsu_datain
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_lsu/lsu_dataout
add wave -noupdate -format Literal /tb_top/top_pss/pippo_core/pippo_lsu/lsu_reg2mem/lsu_op
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_lsu/lsu_reg2mem/memdata
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/pippo_core/pippo_lsu/lsu_reg2mem/regdata
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/clk
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/dat_nonword
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/dimx_ack_o
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/dimx_adr_i
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/dimx_dat_i
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/dimx_dat_o
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/dimx_rqt_i
add wave -noupdate -format Literal /tb_top/top_pss/imx_umc/dimx_sel_i
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/dimx_we_i
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/dmc_ack
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/dmc_rqt_valid
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/iimx_ack_o
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/iimx_adr_i
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/iimx_adr_o
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/iimx_dat_o
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/iimx_rqt_i
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/iimx_rty_o
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/imc_ack
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/imc_rqt_valid
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/rsp_addr
add wave -noupdate -format Literal /tb_top/top_pss/imx_umc/state
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/store_nonword
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/store_nonword_reg
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/uocm_addr
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/uocm_di
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/uocm_we
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/ram_addr
add wave -noupdate -format Logic /tb_top/top_pss/imx_umc/ram_we
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/ram_di
add wave -noupdate -format Literal -radix hexadecimal /tb_top/top_pss/imx_umc/ram_do
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {208100 ps} 0}
configure wave -namecolwidth 168
configure wave -valuecolwidth 65
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
update
WaveRestoreZoom {162700 ps} {284100 ps}
