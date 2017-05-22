/**
 *  @File   SignedDistanceField.h
 *  @brief  util functions for signed distance field
 *  @author Mustafa Mukadam
 *  @date   May 4, 2017
 **/

#include <gpmp2/obstacle/SignedDistanceField.h>


namespace gpmp2 {

/* ************************************************************************** */
void SignedDistanceField::saveSDF(const std::string filename) {
  std::ofstream ofs(filename.c_str());
  assert(ofs.good());
  std::string fext = filename.substr(filename.find_last_of(".") + 1);
  if (fext == "xml") {
    boost::archive::xml_oarchive oa(ofs);
    oa << BOOST_SERIALIZATION_NVP(*this);
  }
  else if (fext == "bin") {
    boost::archive::binary_oarchive oa(ofs);
    oa << *this;
  }
  else {
    boost::archive::text_oarchive oa(ofs);
    oa << *this;
  }
}

/* ************************************************************************** */
void SignedDistanceField::loadSDF(const std::string filename) {
  std::ifstream ifs(filename.c_str());
  if (!ifs.good())
    std::cout<<"File \'"<<filename<<"\' does not exist!"<<std::endl;
  std::string fext = filename.substr(filename.find_last_of(".") + 1);
  if (fext == "xml") {
    boost::archive::xml_iarchive ia(ifs);
    ia >> BOOST_SERIALIZATION_NVP(*this);
  }
  else if (fext == "bin") {
    boost::archive::binary_iarchive ia(ifs);
    ia >> *this;
  }
  else {
    boost::archive::text_iarchive ia(ifs);
    ia >> *this;
  }
}

}
