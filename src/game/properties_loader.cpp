/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  OS 2 Node part for loading game properties
 *========================================================================**/
#include "ecn_baxter/game/data/game_properties.hpp"
#include <ecn_baxter/game/properties_loader.hpp>

namespace ecn_baxter::game {
sptr<data::GameProperties> GamePropertiesLoader::game_props;
sptr<data::GameProperties> GamePropertiesLoader::get_game_props() {
  return game_props;
}

void GamePropertiesLoader::load_file(const std::string &path) {
  std::ifstream handle(path, std::ios::in);

  if (handle.is_open()) {
    //* Read file
    std::stringstream ss;
    std::string line;
    while (!handle.eof()) {
      std::getline(handle, line);
      ss << line;
    }
    handle.close();

    //* Decode JSON
    Document d;
    game_props = std::make_shared<data::GameProperties>();
    game_props->filename = path;
    d.Parse(ss.str().c_str());
    load(d);
  } else {
    std::cout << "File " << path << " not found :c" << std::endl;
    handle.close();
  }
}

void GamePropertiesLoader::load(const Document &d) {
  load_game_name(d);
  load_setup(d);
}

void GamePropertiesLoader::load_game_name(const Document &d) {
  if (!d[data::GameProperties::NAME_TAG].IsString()) {
    std::cout << "[WARN] Can't find \"" << data::GameProperties::NAME_TAG
              << "\" tag for this game !";
    return;
  }
  game_props->game_name = d["name"].GetString();
}

void GamePropertiesLoader::load_setup(const Document &d) {
  if (!d[data::GameProperties::SETUP_TAG].IsObject()) {
    std::cout << "[WARN] Can't find \"" << data::GameProperties::SETUP_TAG
              << "\" tag for this game !";
    return;
  }
  load_setup_points(d);
}

void GamePropertiesLoader::load_setup_points(const Document &d) {
  if (!d[data::GameProperties::SETUP_TAG]
        [data::SetupRequirements::NEEDED_PTS_TAG]
            .IsArray()) {
    std::cout << "[WARN] Can't find \""
              << data::SetupRequirements::NEEDED_PTS_TAG
              << "\" tag for this game !";
    return;
  }
  // Iterate over the points to add
  const Value &pts = d[data::GameProperties::SETUP_TAG]
                      [data::SetupRequirements::NEEDED_PTS_TAG];
  for (auto itr = pts.Begin(); itr != pts.End(); ++itr) {
    const Value &pt = *itr;

    data::GPoint gp;
    gp.name = pt[data::GPoint::NAME_TAG].GetString();
    gp.arm_side = data::bool2side(pt[data::GPoint::ARM_TAG].GetBool());
    gp.w_angles = pt[data::GPoint::ANGLES_TAG].GetBool();
    game_props->setup.needed_points.push_back(gp);
  }
}
} // namespace ecn_baxter::game