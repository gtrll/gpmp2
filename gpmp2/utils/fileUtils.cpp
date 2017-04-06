/**
 *  @file  fileUtils.cpp
 *  @brief utility functions wrapped in matlab
 *  @author Jing Dong
 *  @date  Jan 28, 2017
 **/

#include <gpmp2/obstacle/SignedDistanceField.h>

using namespace gtsam;
using namespace std;


namespace gpmp2 {

/* ************************************************************************** */
bool readSDFvolfile(const std::string& filename_pre, SignedDistanceField& sdf) {

  const string headfilename = filename_pre + ".vol.head";
  ifstream head_file(headfilename.c_str(), ios::in);
  if (!head_file.is_open())
    return false;

  size_t field_rows, field_cols, field_z;
  double origin_x, origin_y, origin_z;
  double vol_res;
  head_file >> field_cols >> field_rows >> field_z;
  head_file >> origin_x >> origin_y >> origin_z;
  head_file >> vol_res;

  cout << field_cols << ", " << field_cols << ", " << field_cols << endl;
  cout << origin_x << ", " << origin_y << ", " << origin_z << endl;
  cout << vol_res << endl;

  head_file.close();

  const string datafilename = filename_pre + ".vol.data";
  ifstream data_file(datafilename.c_str(), ios::in);
  if (!data_file.is_open())
    return false;

  vector<Matrix> vmat;
  for (size_t z = 0; z < field_z; z++) {
    vmat.push_back(Matrix(field_rows, field_cols));
  }

  double tmpsdf;
  for (size_t x = 0; x < field_cols; x++) {
    for (size_t y = 0; y < field_rows; y++) {
      for (size_t z = 0; z < field_z; z++) {
        data_file >> tmpsdf;
        vmat[z](y,x) = tmpsdf;
      }
    }
  }

  data_file.close();

  sdf = SignedDistanceField(Point3(origin_x, origin_y, origin_z), vol_res, vmat);

  return true;
}


}


