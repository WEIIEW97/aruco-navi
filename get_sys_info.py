import pybind11
import sysconfig

def get_python_lib():
    return sysconfig.get_path('stdlib')

def get_python_include():
    return sysconfig.get_path('include')

def get_python_so():
    return sysconfig.get_config_var('LIBDIR')

def get_pybind11_include():
    return pybind11.get_include()

def get_pybind11_cmake():
    return pybind11.get_cmake_dir()

def get_pybind11_pkgconfig():
    return pybind11.get_pkgconfig_dir()

if __name__ == "__main__":
    print(get_python_lib())
    print(get_python_include())
    print(get_python_so())
    print(get_pybind11_include())
    print(get_pybind11_cmake())


