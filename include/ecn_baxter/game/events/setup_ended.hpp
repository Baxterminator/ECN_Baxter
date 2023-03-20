/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  Event launched when the setup phase has ended
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_SETUP_ENDED
#define ECN_BAXTER_SETUP_ENDED

#include <qcoreevent.h>

namespace ecn_baxter::game::events {

class SetupEnded : public QEvent {
public:
  SetupEnded() : QEvent(custom_type) {}

  static QEvent::Type type() { return custom_type; }

private:
  const static QEvent::Type custom_type;
};

} // namespace ecn_baxter::game::events

#endif