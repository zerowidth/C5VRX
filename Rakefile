IDF_IMAGE = "espressif/idf:v6.0.2"
PROJECT = File.read("CMakeLists.txt")[/^project\((\w+)\)/, 1]
# Per-project build dirs and sdkconfig so switching branches never reuses a stale config.
BUILD_DIR = "build-#{PROJECT}"

def serial_port
  return ENV["PORT"] if ENV["PORT"]

  ports = Dir["/dev/cu.usbmodem*"]
  abort "no /dev/cu.usbmodem* found; plug in the board or set PORT" if ports.empty?
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
