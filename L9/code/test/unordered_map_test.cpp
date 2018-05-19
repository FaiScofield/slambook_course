#include <iostream>
#include <unistd.h>
#include <string>
#include <unordered_map>

using namespace std;

int main(int argc, char *argv[])
{

    // unordered_map
     unordered_map<int, string> um;
     um.insert(make_pair<int, string>(0, "eggs"));
     um.insert(make_pair<int, string>(1, "apple"));
     um.insert(make_pair<int, string>(2, "banana"));
     um.insert(make_pair<int, string>(3, "water"));
     um.insert(make_pair<int, string>(4, "desk"));

     for (auto iter=um.begin(); iter!=um.end();) {
         cout << "now: " << iter->first << endl;
         if (iter->first < 2 ) {
             cout << "befor erase: " << iter->first << endl;
             iter = um.erase(iter);
//             cout << "after erase: " << iter->first << endl;
             continue;
         }
         iter++;
     }

     cout << "final size: " << um.size() << endl;


    // vector + pair
//    vector<pair<int, string>>  umap_is;   // all landmarks

//    umap_is.push_back(make_pair<int, string>(0, "eggs"));
//    umap_is.push_back(make_pair<int, string>(1, "apple"));
//    umap_is.push_back(make_pair<int, string>(2, "banane"));
//    umap_is.push_back(make_pair<int, string>(3, "water"));
//    umap_is.push_back(make_pair<int, string>(4, "desk"));

//    for (auto iter=umap_is.begin(); iter!=umap_is.end();) {
//        if (iter->first > 2 ) {
//            cout << "befor erase: " << iter->first << endl;
//            iter = umap_is.erase(iter);
//            cout << "after erase: " << iter->first << endl;

//            continue;
//        }
//        iter++;
//        cout << "now: " << iter->first << endl;

//        if (iter != umap_is.end()) {
//            cout << "not in the end!" << endl;
//        }
//    }

//    cout << "finally size: " << umap_is.size() << endl;

//    pair<int, string> p5 = make_pair<int, string>(5, "cup");
//    pair<int, string> p2 = make_pair<int, string>(2, "bababa");
//    bool in = false;
//    for (auto& u : umap_is) {
//        if (u.first == p5.first) {
//            u.second = p5.second;
//            in = true;
//        }
//    }
//    if (!in) {
//        umap_is.push_back(p5);
//    }
    return 0;
}
