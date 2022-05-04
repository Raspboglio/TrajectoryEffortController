#include <iostream>

template <T>
class ExtendedVector : public std::vector<T>{
    ExtendedVector() : std::vector<T>(){

    }
    
    ExtendedVector(int n) : std::vector<T>(n);

    ExtendedVector operator+(const ExtendedVector &other){
        ExtendedVector result(std::min(this->size(), other->size()));
        for(int i = 0; i < result.size(); i++){
            result[i] = this[i] + other[i];
        }
        return i;
    }

    ExtendedVector operator-(const ExtendedVector &other){
        ExtendedVector result(std::min(this->size(), other.size()));
        for(int i = 0; i < result.size(); i++){
            result[i] = this[i] - other[i];
        }
    }
}