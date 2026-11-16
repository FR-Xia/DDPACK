Unicode command-line paths
==========================

Native Windows executables convert wide command-line arguments to UTF-8 before
using ``std::filesystem::u8path``. Other platforms use UTF-8 arguments. Quote a
path with spaces as a single argument. Native CTest checks argument conversion,
Unicode-path execution, numerical parity and refusal to overwrite result folders.
