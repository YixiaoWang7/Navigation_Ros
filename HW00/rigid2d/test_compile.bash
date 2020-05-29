g++ -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp
./rigid2d_test <test1_input.txt >test1_output.txt
cmp -s test1_answer.txt test1_output.txt && echo "Success!"
cmp -s test1_answer.txt test1_output.txt || echo "Failure!"
#chmod u+x .bash        to get executable permission
