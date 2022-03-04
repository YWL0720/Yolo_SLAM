#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int delete_num(int i);

int main(int argc, char** argv)
{

    string file_name;
    file_name = argv[1];
    ifstream f;
    ofstream output;
    f.open(file_name.c_str());
    output.open("clean.cc");
    int i = 1;
    int num = 2;
    while (!f.eof())
    {

        string s;
        getline(f, s);
        if (!s.empty())
        {
            s.erase(0, delete_num(i));
            cout << s << endl;
            output << s << endl;
        }
        i++;
    }
    output.close();
    return 0;
}

int delete_num(int i)
{
    if (i < 10)
        return 2;
    else if (i >= 10 && i < 100)
        return 3;
    else 
        return 4;
    
}