/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  OS 2 Node part for loading game properties
 *========================================================================**/

#ifndef PROPERTIES_LOADER_HPP
#define PROPERTIES_LOADER_HPP

#include <ecn_baxter/game/data/game_properties.hpp>
#include <ecn_baxter/utils.hpp>
#include <fstream>
#include <iostream>
#include <rapidjson/document.h>
#include <sstream>
#include <string>

namespace ecn_baxter::game {

using rapidjson::Document;
using rapidjson::Value;

class GamePropertiesLoader {
public:
  static sptr<data::GameProperties> get_game_props();

protected:
  static sptr<data::GameProperties> game_props;
  static void load_file(const std::string &);

private:
  static void load(const Document &);
  static void load_game_name(const Document &);
  static void load_setup(const Document &);
  static void load_setup_points(const Document &);
};
} // namespace ecn_baxter::game

#endif // PROPERTIES_LOADER_HPH