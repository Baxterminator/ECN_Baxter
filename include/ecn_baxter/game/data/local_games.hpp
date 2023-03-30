/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  01/03/2023
 * @description    :  Game utils class for game loading
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_LOCAL_GAMES
#define ECN_BAXTER_LOCAL_GAMES

#include "ecn_baxter/game/data/game_properties.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <map>
#include <string>

namespace ecn_baxter::game::data {

typedef std::map<std::string, std::string> GameList;

constexpr auto GAME_EXTENSION{".bgame"};

struct GameUtils {
  //* File Management
  static std::string get_real_path(const std::string &);
  static std::string read_file(const std::string &);

  //* Game Properties
  static bool is_game(const std::string &);
  static std::string get_game_name(const std::string &);
  static GameList get_local_games();

private:
  GameUtils();
};

} // namespace ecn_baxter::game::data
#endif