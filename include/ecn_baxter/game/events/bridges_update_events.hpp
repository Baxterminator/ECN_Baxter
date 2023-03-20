/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  Event launched whenever the bridges list has been updated
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_BRIDGES_EVENT
#define ECN_BAXTER_BRIDGES_EVENT

#include "ecn_baxter/game/data/game_players.hpp"
#include <qcoreevent.h>

namespace ecn_baxter::game::events {

class BridgesUpdate : public QEvent {
public:
  BridgesUpdate(const data::PlayerList &list)
      : QEvent(custom_type), player_list(list) {}

  static QEvent::Type type() { return custom_type; }

  data::PlayerList &get_player_list() { return player_list; }

private:
  const static QEvent::Type custom_type;
  data::PlayerList player_list;
};

} // namespace ecn_baxter::game::events

#endif