# encoding: utf-8
# ruby: 2.4.2
=begin
Rakefile to manage compile CuVoodoo STM32F1 firmware.
the firmware is for development board based around a STM32F1xx micro-controller.
the firmware uses the libopencm3 library providing support for this micro-controller.
=end
require 'rake'
require 'rake/clean'

# the firmwares to compile
BOOTLOADER = "bootloader"
APPLICATION = "application"
FIRMWARES = [BOOTLOADER, APPLICATION]

# which development board is used
# supported are: SYSTEM_BOARD, MAPLE_MINI, BLUE_PILL, CORE_BOARD
BOARD = ENV["BOARD"] || "CORE_BOARD"

# libopencm3 definitions
LIBOPENCM3_DIR = "libopencm3"
LIBOPENCM3_INC = LIBOPENCM3_DIR+"/include"
LIBOPENCM3_LIB = LIBOPENCM3_DIR+"/lib"
# STM32F1 library use for this project provided libtm
STM32F1_LIB = "opencm3_stm32f1"

# source code used by the firmware
SRC_DIRS = [".", "lib"]

# cross-compiler environment
PREFIX = ENV["PREFIX"] || "arm-none-eabi"
#CC = "clang -target #{PREFIX}" # use clang instead of gcc
CC = PREFIX+"-gcc"
#LD = PREFIX+"-ld"
LD = PREFIX+"-gcc"
AR = PREFIX+"-ar"
AS = PREFIX+"-as"
OBJCOPY = PREFIX+"-objcopy"
OBJDUMP = PREFIX+"-objdump"
GDB = PREFIX+"-gdb"

# compiler flags
cflags = [ENV["CFLAGS"]]
# optimize for size
cflags << "-Os"
# add debug symbols (remove for smaller release)
cflags << "-ggdb"
# use C99 (supported by most an sufficient)
cflags << "-std=c99"
# have strict warning (for better code)
cflags << "-Wpedantic -Wall -Werror -Wundef -Wextra -Wshadow -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -Wstrict-overflow=5"
# add options for better code optimization
cflags << "-fno-common -ffunction-sections -fdata-sections"
# use variable size enum (clang doesn't but opencm3, and gcc do)
cflags << "-fshort-enums"
# don't use system main definition (the starting point)
cflags << "-ffreestanding"
# don't use the standard library (only if you provide an alternative libc library)
#cflags << "-nostdlib -nostdinc"
# include own libraries
cflags += SRC_DIRS.collect {|srd_dir| "-I #{srd_dir}"}
# include libopencm3 library
cflags << "-I #{LIBOPENCM3_INC}"
# add defines for micro-controller and board
cflags << "-DSTM32F1 -D#{BOARD}"
# render cflags
cflags = cflags.compact*' '

# linker flags
#ldflags = [ENV["LDFLAGS"]]
ldflags = []
# build static binary (no shared libraries on the micro-controller)
ldflags << "-static"
# don't include the system start files
ldflags << "-nostartfiles"
# linker specific flags
ldflags_linker = [ENV["LDFLAGS"]]
# only keep used sections
#ldflags << "--gc-sections"
ldflags_linker = ["--gc-sections"]
# don't use system libraries (only if you provide an alternative libc library)
#ldflags << "-nostdlib -nostdinc"
# add standard libraries (for libc, libm, libnosys, libgcc) and libopencm3
#library_paths = ["/usr/arm-none-eabi/lib/armv7-m/", "/usr/lib/gcc/arm-none-eabi/*/armv7-m/", LIBOPENCM3_LIB]
# add libopencm3 libraries
library_paths = [LIBOPENCM3_LIB]
# project libraries
ldlibs = [STM32F1_LIB]
#ldflags += library_paths.collect {|library_path| "--library-path #{library_path}"}
#ldflags *= ' '
# used libraries (gcc provides the ARM ABI)
#ldlibs = [STM32F1_LIB, "c", "m", "nosys", "gcc"]
#ldlibs = ldlibs.collect {|library| "--library #{library}"}
#ldlibs *= ' '
ldlibs_linker = ["m", "c", "nosys", "gcc"]

# target micro-controller information (ARM Cortex-M3 supports thumb and thumb2, but does not include a floating point unit)
archflags = "-mthumb -mcpu=cortex-m3 -msoft-float"

desc "compile firmwares"
task :default => FIRMWARES
task :compile => FIRMWARES

FIRMWARES.each do |firmware|
  desc "compile #{firmware} firmware"
  task firmware => [firmware+".elf", firmware+".bin", firmware+".hex"]
  CLOBBER.include(firmware+".elf")
  CLOBBER.include(firmware+".bin")
  CLOBBER.include(firmware+".hex")
  CLOBBER.include(firmware+".list")
  CLOBBER.include(firmware+".map")
end

# get dependencies of a file
# done is a list of already known dependencies
def dependencies(source, done=[])
  d_path = source.ext("d") # get the dependency file
  Rake::Task[d_path].invoke # ensure the dependency file exists
  d_file = IO.read(d_path) # read the dependencies from dependency file
  d_file = d_file.split(': ')[1].gsub("\n",'').gsub('\\ ','').gsub(/\s+/,' ').split(' ') # get a list of dependencies
  d_list = [] # list of dependencies
  # only save dependencies which are in our source directories
  d_file.each do |d|
    SRC_DIRS.each do |dir|
      if File.dirname(d)==dir then
        d_list << d
      end
    end
  end
  # get the dependencies of these dependencies, if we don't know them already
  done << source.ext("o")
  done.uniq!
  d_list.each do |d|
    d = d.ext("o")
    next if done.include? d
    done += dependencies(d, done)
  end
  done.uniq!
  return done
end

desc "get libopencm3"
file LIBOPENCM3_DIR+"/Makefile" do
  sh "git submodule init"
  sh "git submodule update"
end

desc "compile libopencm3"
file "#{LIBOPENCM3_LIB}/lib#{STM32F1_LIB}.a" => LIBOPENCM3_DIR+"/Makefile" do
  sh "make --directory #{LIBOPENCM3_DIR}"
end

task :doc => ["Doxyfile", "README.md"] do |t|
  sh "doxygen #{t.source}"
end

desc "compile source into object"
rule '.o' => '.c' do |t|
  sh "#{CC} #{cflags} #{archflags} -o #{t.name} -c #{t.source}"
end

desc "generate dependencies"
rule '.d' => '.c' do |t|
  sh "#{CC} #{cflags} #{archflags} -MM -MF #{t.name} -c #{t.source}"
end

#desc "link binary"
#rule '.elf' => [proc{|f| dependencies(f)}, '.ld', "#{LIBOPENCM3_LIB}/lib#{STM32F1_LIB}.a"] do |t|
#  sh "#{LD} #{ldflags} --script #{t.name.ext('ld')} #{t.prerequisites[0..-3].join(' ')} #{ldlibs} -o #{t.name}"
#  sh "size #{t.name}"
#end

desc "link binary"
rule '.elf' => ['.o', proc{|f| dependencies(f)}, '.ld', "#{LIBOPENCM3_LIB}/lib#{STM32F1_LIB}.a"] do |t|
  sh "#{LD} #{archflags} #{ldflags.join(' ')} #{t.prerequisites[0..-3].join(' ')} -T#{t.name.ext('ld')} #{ldflags_linker.collect{|flag| "-Wl,"+flag}.join(' ')} #{library_paths.collect{|path| "-L"+path}.join(' ')} #{ldlibs.collect{|lib| "-l"+lib}.join(' ')} -Wl,--start-group #{ldlibs_linker.collect{|lib| "-l"+lib}.join(' ')} -Wl,--end-group --output #{t.name}"
  sh "size #{t.name}"
end

desc "export binary"
rule '.bin' => '.elf' do |t|
  sh "#{OBJCOPY} --strip-all --strip-debug --output-target binary #{t.source} #{t.name}"
end

desc "export intel hex file"
rule '.hex' => '.elf' do |t|
  sh "#{OBJCOPY} --strip-all --strip-debug --output-target ihex #{t.source} #{t.name}"
end

desc "export list"
rule '.list' => '.elf' do |t|
  sh "#{OBJDUMP} -S #{t.source} > #{t.name}"
end

desc "export map"
rule '.map' => '.elf' do |t|
  sh "#{OBJDUMP} -S #{t.source} > #{t.name}"
end

SRC_DIRS.each do |src_dir|
  CLEAN.include(src_dir+"/*.o")
  CLEAN.include(src_dir+"/*.d")
end

# SWD/JTAG adapter used
# supported are : STLINKV2 (ST-Link V2), BMP (Black Magic Probe)
SWD_ADAPTER = ENV["SWD_ADAPTER"] || "BMP"
# openOCD path to control the adapter
OOCD = ENV["OOCD"] || "openocd"
# openOCD adapted name
OOCD_INTERFACE = ENV["OOCD_INTERFACE"] || (SWD_ADAPTER=="STLINKV2" ? "stlink-v2" : "")
# openOCD target for the micro-controller
OOCD_TARGET = "stm32f1x"
# Black Magic Probe port
BMP_PORT = ENV["BMP_PORT"] || "/dev/ttyACM0"

desc "flash application using USB DFU"
task :flash => APPLICATION+".bin" do |t|
  sh "dfu-util -d c440:0d00 -D #{t.source}"
end

desc "flash bootloader using SWD"
task :flash_bootloader => BOOTLOADER+".hex" do |t|
  case SWD_ADAPTER
    when "STLINKV2"
      sh "#{OOCD} --file interface/#{OOCD_INTERFACE}.cfg --file target/#{OOCD_TARGET}.cfg --command 'init' --command 'reset init' --command 'flash write_image erase #{t.source}' --command 'reset' --command 'shutdown'"
    when "BMP"
      sh "#{GDB} --eval-command='target extended-remote #{BMP_PORT}' --eval-command='set confirm off' --eval-command='monitor swdp_scan' --eval-command='attach 1' --eval-command='load' --eval-command='kill' --eval-command='quit' #{t.source}"
  end
end

desc "flash application using SWD"
task :flash_application => APPLICATION+".hex" do |t|
  case SWD_ADAPTER
    when "STLINKV2"
      sh "#{OOCD} --file interface/#{OOCD_INTERFACE}.cfg --file target/#{OOCD_TARGET}.cfg --command 'init' --command 'reset init' --command 'flash write_image erase #{t.source}' --command 'reset' --command 'shutdown'"
    when "BMP"
      sh "#{GDB} --eval-command='target extended-remote #{BMP_PORT}' --eval-command='set confirm off' --eval-command='monitor swdp_scan' --eval-command='attach 1' --eval-command='load' --eval-command='kill' --eval-command='quit' #{t.source}"
  end
end

# debug application using GDB
desc "debug application using GDB"
task :debug => APPLICATION+".elf" do |t|
  case SWD_ADAPTER
    when "STLINKV2"
      # for GDB to work with openOCD the firmware needs to be reloaded
      sh "#{GDB} --eval-command='target remote | #{OOCD} --file interface/#{OOCD_INTERFACE}.cfg --file target/#{OOCD_TARGET}.cfg --command \"gdb_port pipe; log_output /dev/null; init\"' --eval-command='monitor reset halt' --eval-command='load' --eval-command='monitor reset init' #{t.source}"
    when "BMP"
      sh "#{GDB} --eval-command='target extended-remote #{BMP_PORT}' --eval-command='monitor version' --eval-command='monitor swdp_scan' --eval-command='attach 1' #{t.source}"
  end
end

# reset device by setting the data width to 5 bis on the USB CDC ACM port
ACM_PORT = ENV["ACM_PORT"] || ("/dev/ttyACM0"==BMP_PORT ? "/dev/ttyACM2" : "/dev/ttyACM0")

desc "reset application using serial"
task :reset do
  sh "stty --file #{ACM_PORT} raw cs5"
  sleep 0.5
end
