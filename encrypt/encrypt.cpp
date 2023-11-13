#include <fstream>
#include <iostream>
using namespace std;


int encrypt(string orig_fn, string new_fn) {
    int key = 5;
    char c;
    
    //input stream
    fstream fin, fout;

    fin.open(orig_fn, fstream::in);
    fout.open(new_fn, fstream::out);

    while (fin >> noskipws >> c){
        int temp = (c + key);
        fout << (char)temp;
    } // while

    fin.close();
    fout.close();
    
    return 0;
} // encrypt()


int main (int argc, char* argv[]){
    if (argc < 2){ // not enough arguments
        cout << "Please include a file name when running the program.";
        return 1;
    } else if (argc > 2){ // to many arguments
        cout << "Too many arguments.";
        return 1;
    } else { // perfect!
        string fn = argv[1];
        
        string f_ext = fn.substr(fn.length()-4, -1);
        //cout << fileext;
        
        if (f_ext != ".txt"){ // does not end in .txt
            cout << "Only accepts '.txt' files.";
            return 1;
        } else { // we have 1 text file!
            // now we encrypt the file
            string new_fn = fn.substr(0, fn.length()-4) + "-encrypted" + f_ext;
            
            encrypt(fn, new_fn);
        } 
    }
    //cout << "Hello @?";
    return 0;

} // int main()