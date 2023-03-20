/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  01/03/2023
 * @description    :  Game utils class for game loading
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/data/local_games.hpp"
#include "ecn_baxter/package_data.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <rapidjson/document.h>

namespace ecn_baxter::game::data {
namespace fs = std::filesystem;
using rapidjson::Document;

/**════════════════════════════════════════════════════════════════════════
 *!                           FILE MANAGEMENT
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief get the real path of a file (to remove symlink)
std::string GameUtils::get_real_path(const std::string &path) {
  return (fs::is_symlink(path)) ? (std::string)fs::read_symlink(path) : path;
}

/// @brief read a whole file and returning a string
std::string GameUtils::read_file(const std::string &path) {
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
    return ss.str();
  } else {
    std::cout << "File " << path << " not found :c" << std::endl;
    handle.close();
    return "";
  }
}

/**════════════════════════════════════════════════════════════════════════
 *!                           GAME PROPERTIES
 * ════════════════════════════════════════════════════════════════════════**/
 
/// @brief test if a file a .bgame file
bool GameUtils::is_game(const std::string &path) {
  // Check if file exist
  if (!fs::is_regular_file(path))
    return false;

  // Check file extension
  auto ext = path.substr(path.length() - 6, 6);
  if (ext != GAME_EXTENSION)
    return false;
  return true;
}

/// @brief load the name of a game from his .bgame file
std::string GameUtils::get_game_name(const std::string &path) {
  if (!is_game(path))
    return "";

  // Else read the file and decode it
  Document data;
  data.Parse(read_file(path).c_str());
  if (data[GameProperties::NAME_TAG].IsString())
    return data[GameProperties::NAME_TAG].GetString();
  return "";
}

/// @brief Return a map of the games present in the package folder associated
/// to their paths
GameList GameUtils::get_local_games() {
  using ament_index_cpp::get_package_share_directory;
  GameList list;

  auto share_dir = get_package_share_directory(PACKAGE_NAME) + GAMES_DIR;
  for (auto &entry : fs::directory_iterator(share_dir)) {
    auto real_path = get_real_path(entry.path());
    std::string name;
    if (is_game(real_path) && (name = get_game_name(real_path)) != "") {
      list.insert({name, real_path});
    }
  }

  return list;
};

} // namespace ecn_baxter::game::data