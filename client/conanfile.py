from conans import ConanFile, CMake

class GlenniferClientConan(ConanFile):
   settings = "os", "compiler", "build_type", "arch"
   requires = "librabbitmq/0.8.1@filonovpv/stable", "amqp/1.0-snapshot@ichaelm/testing", "yaml-cpp/0.5.3@moonforged/stable", "OpenCV/3.2.0@ohhi/stable"
   generators = "qmake"
   build_policy = "missing"
   
   def config(self):
      if self.settings.os == "Windows":
         self.requires("Protobuf/2.6.1@memsharded/testing")
   
   def imports(self):
      if self.settings.os == "Windows":
         self.copy("*.exe", dst="bin", src="bin")
         self.copy("*.dll", dst="bin", src="bin")
         self.copy("*.dll", dst="bin", src="lib")
