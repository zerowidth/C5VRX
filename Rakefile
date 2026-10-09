IDF_IMAGE = "espressif/idf:v6.0.2"
PROJECT = File.read("CMakeLists.txt")[/^project\((\w+)\)/, 1]
# Per-project build dirs and sdkconfig so switching branches never reuses a stale config.
BUILD_DIR = "build-#{PROJECT}"

IDF_PYTHON = File.expand_path("~/.espressif/tools/python/v6.1/venv/bin/python")

# USB IDs (VID:PID) of the serial ports each task can talk to.
USB_SERIAL_JTAG = "303a:1001" # C5 or S3 native USB
P4_CONSOLE = "303a:8000"      # p4usb's CDC console on the P4's high-speed port
P4_ROM = "303a:0012"          # the P4's ROM loader on the same port
P4_UART = "1a86:55d3"         # the CH343 on the P4-Pico's USB-C

# Serial ports as [device, "vid:pid"], read through pyserial from ESP-IDF's Python.
def usb_ports
  script = <<~PY
    from serial.tools.list_ports import comports
    for p in comports():
        if p.vid is not None:
            print(p.device, "%04x:%04x" % (p.vid, p.pid))
  PY
  IO.popen([IDF_PYTHON, "-c", script], &:read).lines.map(&:split)
end

# The one port matching the first USB ID that has any, or PORT when set.
def port_for(name, *ids)
  return ENV["PORT"] if ENV["PORT"]

  ports = usb_ports
  ids.each do |id|
    found = ports.select { |_, i| i == id }.map(&:first)
    next if found.empty?
    abort "several #{name} ports (#{found.join(", ")}); set PORT" if found.size > 1
    return found.first
  end
  abort "no #{name} port found; plug it in or set PORT"
end

def wait_for_port(name, id, seconds: 10)
  deadline = Time.now + seconds
  until Time.now > deadline
    found = usb_ports.find { |_, i| i == id }
    return found.first if found
    sleep 0.2
  end
  abort "#{name} did not appear within #{seconds} s (a new USB device may need approving in macOS)"
end

desc "Build the firmware with ESP-IDF in Docker"
task :build do
  sh "docker", "run", "--rm", "-v", "#{Dir.pwd}:/workspace", "-w", "/workspace", IDF_IMAGE,
     "idf.py", "-B", BUILD_DIR, "-D", "SDKCONFIG=#{BUILD_DIR}/sdkconfig", "build"
end

desc "Build, then flash over USB (PORT=/dev/cu.usbmodemXXXX to choose a port)"
task flash: :build do
  Dir.chdir(BUILD_DIR) do
    sh "esptool", "--chip", "esp32c5", "-p", port_for("ESP32-C5", USB_SERIAL_JTAG), "-b", "460800",
       "--before", "default-reset", "--after", "hard-reset", "write-flash", "@flash_args"
  end
end

desc "Remove the build directory"
task :clean do
  rm_rf BUILD_DIR
end

task default: :build

IDF_ACTIVATE = File.expand_path("~/.espressif/tools/activate_idf_v6.1.sh")

# Runs a command in dir under the local ESP-IDF v6.1, where idf.py is a shell
# function. The activate script is sourced with no arguments so it can't misread ours.
def local_idf(dir, *cmd)
  sh "bash", "-c", 'args=("$@"); set --; . "${args[0]}" >/dev/null; cd "${args[1]}" && "${args[@]:2}"',
     "bash", IDF_ACTIVATE, dir, *cmd
end

def p4_console_port
  port_for("P4 console", P4_CONSOLE, P4_UART)
end

# esptool against the C5 through the p4usb bridge, which resets the C5 into
# download mode when it sees esptool's SYNC and follows esptool's baud change.
def c5_via_p4(dir, *args)
  local_idf dir, "esptool", "--chip", "esp32c5", "-p", p4_console_port, "-b", ENV.fetch("BAUD", "921600"),
            "--before", "no-reset", "--after", "watchdog-reset", *args
end

# The P4's ROM loader port on its high-speed USB, rebooting p4usb into it with
# esptool's reset sequence if needed. Nil when only the USB-C UART is available.
def p4_rom_port
  return nil if ENV["PORT"]

  ports = usb_ports
  rom = ports.find { |_, i| i == P4_ROM }
  return rom.first if rom

  console = ports.find { |_, i| i == P4_CONSOLE }
  return nil unless console

  script = <<~PY
    import sys, time, serial
    s = serial.Serial(sys.argv[1])
    s.dtr = False; s.rts = True; time.sleep(0.1)
    s.dtr = True; s.rts = False; time.sleep(0.05)
    s.close()
  PY
  system(IDF_PYTHON, "-c", script, console.first) or abort "could not reset the P4 console"
  wait_for_port("P4 ROM loader", P4_ROM)
end

namespace :p4usb do
  desc "Build the P4 firmware with the local ESP-IDF"
  task :build do
    local_idf "p4usb", "idf.py", "build"
  end

  desc "Build, then flash the P4 over its high-speed USB, or its USB-C if that is all there is"
  task flash: :build do
    if (rom = p4_rom_port)
      local_idf "p4usb/build", "esptool", "--chip", "esp32p4", "-p", rom, "--before", "no-reset",
                "--after", "watchdog-reset", "write-flash", "@flash_args"
    else
      local_idf "p4usb", "idf.py", "-p", port_for("P4 USB-C", P4_UART), "flash"
    end
  end

  desc "Open the P4's boot and panic log on its USB-C"
  task :monitor do
    local_idf "p4usb", "idf.py", "-p", port_for("P4 USB-C", P4_UART), "monitor"
  end

  desc "Open the P4 console in picocom without resetting the P4 (exit with C-a C-x)"
  task :console do
    sh "picocom", "-b", "115200", "--imap", "lfcrlf", p4_console_port
  end

  desc "Read the C5's flash ID through the P4 bridge"
  task :c5_flash_id do
    c5_via_p4 ".", "flash-id"
  end

  desc "Print the P4's chip revision"
  task :chip_id do
    local_idf "p4usb", "esptool", "--chip", "esp32p4", "-p", port_for("P4 USB-C", P4_UART), "chip-id"
  end
end

namespace :c5rx do
  desc "Build the C5 firmware for the P4 receiver with the local ESP-IDF"
  task :build do
    local_idf "c5rx", "idf.py", "build"
  end

  desc "Build, then flash the C5 through the P4 bridge"
  task flash: :build do
    c5_via_p4 "c5rx/build", "write-flash", "@flash_args"
  end
end
