/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  Register all the custom events types
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/events/auth_refresh_event.hpp"
#include "ecn_baxter/game/events/bridges_update_events.hpp"
#include "ecn_baxter/game/events/log_event.hpp"
#include "ecn_baxter/game/events/setup_ended.hpp"
#include "ecn_baxter/game/utils/qtevents.hpp"
#include <qcoreevent.h>

namespace ecn_baxter::game::events {

using Type = QEvent::Type;
#define new_type() static_cast<Type>(QEvent::registerEventType())

// Define the custom events here
const Type BridgesUpdate::custom_type = new_type();
const Type SetupEnded::custom_type = new_type();
const Type LogEvent::custom_type = new_type();
const Type AuthRefresh::custom_type = new_type();

} // namespace ecn_baxter::game::events