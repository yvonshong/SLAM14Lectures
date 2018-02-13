 ../../../../wonder-ar/externals/protobuf/3.3.2/windows_x64_vc14/shared/release/bin/protoc --cpp_out=../src/client --grpc_out=../src/client --plugin=protoc-gen-grpc=" ../../../../wonder-ar/externals/grpc/1.4.2/windows_x64_vc14/shared/release/bin/grpc_cpp_plugin.exe"  netVlad.proto

python -m grpc_tools.protoc -I. --python_out=../src/server --grpc_python_out=../src/server  ./netVlad.proto
