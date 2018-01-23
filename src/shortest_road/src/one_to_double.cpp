#include<iostream>
#include<fstream>
#include<vector>
#include<set>
#include<stdlib.h>
#define LEN 7.5

typedef struct{
    std::string from;
    std::string to;
    float dist;
}record_t;


std::vector<record_t> one;
std::vector<record_t> doubles;
//以c中包含的字符对s进行切分
void splits(const std::string& s, std::vector<std::string>& v, const std::string& c){
    std::string::size_type pos1, pos2;
    std::set<char> splits;
    int i = 0;
    while(c[i])
        splits.insert(c[i++]);

    pos1 = 0;
    i = 0;
    while(s[i]){
        if(splits.count(s[i]) ){
            pos2 = i;
            if(pos2-pos1 > 0)
                v.push_back(s.substr(pos1, pos2-pos1));
            pos1 = pos2+1;
        }
        i++;
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

void readFromOne(const std::string &filename){
    // open the file  
	std::ifstream inFile( filename.c_str(), std::ios_base::in);
	if (!inFile.is_open()){
		std::cout<<"file read failed "<<filename;
		return ;
	}
    std::string line;
    getline(inFile, line);
    
    std::vector<std::string> v;
    record_t record;
    while(getline(inFile, line)){
        splits(line, v, ",");
        record.from = v[0];
        record.to = v[1];
        record.dist = atof(v[2].c_str());
        one.push_back(record);
        v.clear();
    }
}
void oneToDoule(){
    unsigned int len = one.size();
    unsigned int index = 0;
    unsigned int record_len1;
    unsigned int record_len2;
    record_t r;
    for(unsigned int i = 0; i < len; i++){
        record_len1 = one[i].from.size();
        record_len2 = one[i].to.size();
        if(one[i].from[record_len1-1] == '1' &&
            one[i].from[record_len1-2] == '-'){
            r.from = one[i].from;
            r.from[record_len1-1] = '2';
            r.to = one[i].to;
            r.dist = one[i].dist + LEN;//LEN
            doubles.push_back(r);//2-1

            record_len1 = one[i].to.size();
            if(one[i].to[record_len1-1] == '1' &&
                one[i].to[record_len1-2] == '-'){
                    r.to = one[i].to;
                    r.to[record_len1-1] = '2';
                    r.dist += LEN;//2*LEN 2-2
                    doubles.push_back(r);//2-2

                    r.from = one[i].from;
                    r.dist -= LEN;//LEN 1-2
                    doubles.push_back(r);//1-2
                }
        }
        else if(one[i].to[record_len2-1] == '1' &&
                one[i].to[record_len2-2] == '-'){
                r.from = one[i].from;
                r.to = one[i].to;
                r.to[record_len2-1] = '2';
                r.dist = one[i].dist + LEN;//2*LEN 2-2
                doubles.push_back(r);//2-2
        }

        doubles.push_back(one[i]);//1-1
    }
}
void writeToDouble(const std::string &filename){
    std::ofstream ofile(filename.c_str());
    if(!ofile.is_open()){
        std::cout<<"can not open file "<<filename;
        return ;
    }
    ofile<<"from_position,to_position,distance"<<std::endl;
    // std::map<std::string,TargetX> __targets;
    unsigned int len = doubles.size();
    for(unsigned int i = 0; i < len ; i++)
    {
        ofile<<doubles[i].from<<","<<doubles[i].to<<","<<doubles[i].dist<<std::endl;
    }
    ofile.close();
    return ;
}
int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        std::cout<<"Usage: cmd file"<<std::endl;
        return -1;
    }
    readFromOne(std::string(argv[1]));
    oneToDoule();
    writeToDouble(std::string(argv[2]));

    return 0;
}