# Working workflow
rm -rf `pwd`/build/coverage/*
LLVM_PROFILE_FILE="`pwd`/build/coverage/vmltest-validity-avx.profraw" `pwd`/build/unit_tests/vmltest-validity-avx
LLVM_PROFILE_FILE="`pwd`/build/coverage/vmltest-validity-sse.profraw" `pwd`/build/unit_tests/vmltest-validity-sse
LLVM_PROFILE_FILE="`pwd`/build/coverage/vmltest-validity-sse3.profraw" `pwd`/build/unit_tests/vmltest-validity-sse3
LLVM_PROFILE_FILE="`pwd`/build/coverage/vmltest-validity-cpp.profraw" `pwd`/build/unit_tests/vmltest-validity-cpp
llvm-profdata merge -output=`pwd`/build/coverage/code.profdata `pwd`/build/coverage/vmltest-*.profraw
llvm-cov show `pwd`/build/unit_tests/vmltest-validity-sse3 -instr-profile=`pwd`/build/coverage/code.profdata `pwd`/include/*.* -path-equivalence -use-color --format html > `pwd`/build/coverage/coverage.html
llvm-cov export `pwd`/build/unit_tests/vmltest-validity-sse3 -instr-profile=`pwd`/build/coverage/code.profdata `pwd`/include/*.* -path-equivalence -use-color --format lcov > `pwd`/build/coverage/coverage.lcov

genhtml --prefix `pwd`/build/unit_tests/vmltest-validity-avx --ignore-errors source `pwd`/build/coverage/coverage.lcov \
--legend --title "Coverage" --output-directory=`pwd`/build/coverage
