#include <fstream>
#include <iostream>
#include <string>
using namespace std;


int decrypt(string orig_fn, string new_fn, int key){
    char c;
    
    fstream fin;
    fstream fout;
    fin.open(orig_fn, fstream::in);
    fout.open(new_fn, fstream::out);
    
    while (fin >> noskipws >> c) {
        int temp = (c - key);
        fout << (char)temp;
    }// while
    
    fin.close();
    fout.close();
    return 0;
}


int main (int argc, char* argv[]){
    if (argc < 3){ // not enough arguments
        cout << "Please include a file name and a key when running the program.";
        return 1;
    } else if (argc > 3){ // to many arguments
        cout << "Too many arguments.";
        return 1;
    }

    string fn = argv[1]; 
    string fn_ext = fn.substr(fn.length()-4, -1);
    //cout << fn_ext;
    if (fn_ext != ".txt") { // fn is not a .txt file
        cout << "Please enter a .txt file before the key.";
    } 
    /*
    else if (fn.find("-encrypted") == -1){ // the file being passed in has not been encrypted
        cout << "The file being passed in has not been encrypted.";
        return 1;
    }
    */

    string decrypt_fn = fn.substr(0, 4) + "-decrypted" + fn_ext; 
    cout << decrypt_fn;

    decrypt(fn, decrypt_fn, stoi(argv[2]));
    
    //string decrypt_fn = 





    //cout << "Hello @?";
    return 0;

} // int main()