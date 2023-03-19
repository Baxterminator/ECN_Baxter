/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  Register all the custom events types
 *========================================================================**/
#include "ecn_baxter/game/events/bridges_update_events.hpp"
#include <qcoreevent.h>

namespace ecn_baxter::game::events {

using Type = QEvent::Type;

#define new_type() static_cast<Type>(QEvent::registerEventType())

const Type BridgesUpdate::custom_type = new_type();

} // namespace ecn_baxter::game::events