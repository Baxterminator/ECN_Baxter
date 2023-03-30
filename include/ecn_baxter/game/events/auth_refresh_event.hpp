/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  Event launched whenever the bridges list has been updated
 * @version        :  rev 23w12.4
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_AUTH_REFRESH_EVENT
#define ECN_BAXTER_AUTH_REFRESH_EVENT

#include <qcoreevent.h>

namespace ecn_baxter::game::events {

class AuthRefresh : public QEvent {
public:
  AuthRefresh() : QEvent(custom_type) {}

  static QEvent::Type type() { return custom_type; }

private:
  const static QEvent::Type custom_type;
};

} // namespace ecn_baxter::game::events

#endif