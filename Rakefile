IDF_IMAGE = "espressif/idf:v6.0.2"
PROJECT = File.read("CMakeLists.txt")[/^project\((\w+)\)/, 1]
# Per-project build dirs and sdkconfig so switching branches never reuses a stale config.
BUILD_DIR = "build-#{PROJECT}"

def serial_port
  return ENV["PORT"] if ENV["PORT"]

  ports = Dir["/dev/cu.usbmodem*", "/dev/cu.wchusbserial*"]
  abort "no USB serial port found; plug in the board or set PORT" if ports.empty?
  abort "several ports found (#{ports.join(", ")}); set PORT" if ports.size > 1
  ports.first
end

desc "Build the firmware with ESP-IDF in Docker"
task :build do
  sh "docker", "run", "--rm", "-v", "#{Dir.pwd}:/workspace", "-w", "/workspace", IDF_IMAGE,
     "idf.py", "-B", BUILD_DIR, "-D", "SDKCONFIG=#{BUILD_DIR}/sdkconfig", "build"
end

desc "Build, then flash over USB (PORT=/dev/cu.usbmodemXXXX to choose a port)"
task flash: :build do
  port = serial_port
  Dir.chdir(BUILD_DIR) do
    sh "esptool", "--chip", "esp32c5", "-p", port, "-b", "460800",
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

# esptool against the C5 through the p4usb bridge, which resets the C5 into
# download mode when it sees esptool's SYNC and follows esptool's baud change.
def c5_via_p4(dir, *args)
  local_idf dir, "esptool", "--chip", "esp32c5", "-p", serial_port, "-b", ENV.fetch("BAUD", "921600"),
            "--before", "no-reset", "--after", "watchdog-reset", *args
end

namespace :p4usb do
  desc "Build the P4 firmware with the local ESP-IDF"
  task :build do
    local_idf "p4usb", "idf.py", "build"
  end

  desc "Build, then flash the P4 over its USB-C"
  task flash: :build do
    local_idf "p4usb", "idf.py", "-p", serial_port, "flash"
  end

  desc "Open the P4 console (it also relays the C5's)"
  task :monitor do
    local_idf "p4usb", "idf.py", "-p", serial_port, "monitor"
  end

  desc "Open the P4 console in picocom without resetting the P4 (exit with C-a C-x)"
  task :console do
    sh "picocom", "-b", "115200", "--imap", "lfcrlf", serial_port
  end

  desc "Read the C5's flash ID through the P4 bridge"
  task :c5_flash_id do
    c5_via_p4 ".", "flash-id"
  end

  desc "Print the P4's chip revision"
  task :chip_id do
    local_idf "p4usb", "esptool", "--chip", "esp32p4", "-p", serial_port, "chip-id"
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
