@echo off
set xv_path=C:\\dev_tools\\Xilinx\\Vivado\\2017.2\\bin
call %xv_path%/xelab  -wto f8c74caa1c264706b767816e4e7fa478 -m64 --debug typical --relax --mt 2 -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip --snapshot mmult_tb_behav xil_defaultlib.mmult_tb xil_defaultlib.glbl -log elaborate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
