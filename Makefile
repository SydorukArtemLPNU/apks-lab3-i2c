# Lab 3 (variant 20): I2C master/slave in Verilog
# Tools: Icarus Verilog (simulation), Yosys + nextpnr-ice40 (synthesis, P&R, timing)
RTL  := rtl/i2c_master.v rtl/i2c_slave.v
TB   := tb/tb_i2c.v
PY   ?= python3
DEV  ?= --up5k --package sg48
FREQ ?= 12

.PHONY: all sim waves syn clean
all: sim syn

sim: build/tb_i2c.vvp                 ## run the self-checking testbench
	cd build && vvp -n tb_i2c.vvp | tee sim.log
	@grep -q "TESTS PASSED" build/sim.log

build/tb_i2c.vvp: $(RTL) $(TB)
	@mkdir -p build
	iverilog -g2012 -Wall -o $@ $(RTL) $(TB)

waves: sim                            ## render timing diagrams to docs/img
	$(PY) tools/plot_vcd.py build/tb_i2c.vcd docs/img/wave_start_addr.png 1 34 --decode
	$(PY) tools/plot_vcd.py build/tb_i2c.vcd docs/img/wave_repstart_read.png 163 252 --decode

syn: build/syn/i2c_top_pnr.log build/syn/slave_top_pnr.log   ## synthesis + P&R for iCE40 UP5K
	@grep -h "Max frequency" build/syn/*_pnr.log

build/syn/%_pnr.log: syn/%.v $(RTL)
	@mkdir -p build/syn
	yosys -q -l build/syn/$*_yosys.log -p "read_verilog $(RTL) syn/$*.v; synth_ice40 -top $* -json build/syn/$*.json; stat"
	nextpnr-ice40 $(DEV) --json build/syn/$*.json --freq $(FREQ) --asc build/syn/$*.asc \
	    --report build/syn/$*_report.json --log $@

clean:
	rm -rf build
