```cpp
int main(){
    ifstream ifile("../lidardata.pcap",ios::binary);
    if(!ifile){
        cerr<<"open failed"<<endl;
        return 0;
    }

    ifile.seekg(0,ifile.end);
    int length = ifile.tellg();
    ifile.seekg(0,ifile.beg);
    cout<<"file length: "<<length<<" bytes"<<endl;

    unsigned char buf[16];

    ofstream ofile("o.dat",ios::binary);
    for(int i=0;i<10;i++){
        //cout<<ifile.get()<<endl;
        //ofile<<ifile.get();
        buf[i] = ifile.get();
        ofile<<buf[i];
    }

    return 0;
}
```

lidardata.pcap:
```
d4c3 b2a1 0200 0400 0000 0000 0000 0000
ffff 0000 0100 0000 b242 a25a a0a0 0a00
1205 0000 1205 0000 ffff ffff ffff 000a
```

o.dat:
```
d4c3 b2a1 0200 0400 0000 
```

ok, no problem, but when

```cpp
for(int i=0;i<10;i++){
        //cout<<ifile.get()<<endl;
        ofile<<ifile.get();
        //buf[i] = ifile.get();
        //ofile<<buf[i];
}
```

o.dat:
```
212195178161204000
```