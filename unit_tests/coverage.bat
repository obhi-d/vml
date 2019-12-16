call mkdir coverage
call OpenCppCoverage --sources=..\..\..\.. --modules=vmltest-* --export_type=binary:sse.cov -- vmltest-validity-sse.exe
call OpenCppCoverage --sources=..\..\..\.. --modules=vmltest-* --export_type=binary:cpp.cov -- vmltest-validity-cpp.exe
call OpenCppCoverage --sources=..\..\..\.. --modules=vmltest-* --export_type=binary:avx.cov -- vmltest-validity-avx.exe
call OpenCppCoverage --sources=..\..\..\.. --modules=vmltest-* --export_type=html:coverage --input_coverage=cpp.cov --input_coverage=sse.cov --input_coverage=avx.cov -- vmltest-validity-sse3.exe

