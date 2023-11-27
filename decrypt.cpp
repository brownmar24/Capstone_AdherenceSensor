#include <fstream>
#include <iostream>
#include <string>
//#include <Speck.h>
using namespace std;


int decrypt(string orig_fn, string new_fn, int key){
    char c;
    
    fstream fin;
    fstream fout;
    fin.open(orig_fn, fstream::in);
    fout.open(new_fn, fstream::out);
    
    while (fin >> noskipws >> c) {
        //print encrypted char
        //cout << c;

        int temp = (c - key);
        fout << (char)temp;

        //print decrypted char
        //cout << (char)temp;
    }// while
    
    fin.close();
    fout.close();
    return 0;
}


int main (int argc, char* argv[]){
    // error checking for number of inputs
    if (argc < 3){ // not enough arguments
        cout << "Please include a file name and a key when running the program.";
        return 1;
    } else if (argc > 3){ // to many arguments
        cout << "Too many arguments.";
        return 1;
    }

    //error checking for input file type
    string fn = argv[1];
    string fn_ext = fn.substr(fn.length()-4, -1);
    if (fn_ext != ".txt") { // fn is not a .txt file
        cout << "Please enter a .txt file before the key.";
    } 

    string decrypt_fn = fn.substr(0, 4) + "-decrypted" + fn_ext; 

    decrypt(fn, decrypt_fn, stoi(argv[2]));
    //cout << stoi(argv[2]);
    //string decrypt_fn = 





    //cout << "Hello @?";
    return 0;

} // int main()