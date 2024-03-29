/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  QTEvent utils functions
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_UTILS_QTEVENTS
#define ECN_BAXTER_UTILS_QTEVENTS

#include <map>
#include <qcoreevent.h>
#include <string>

namespace ecn_baxter::utils::qt {

/// @brief Return a string equivalent of the event type
inline std::string get_qtevent_name(QEvent *ev) {
  switch (ev->type()) {
  case QEvent::None:
    return "QEvent::None";
  case QEvent::ActionAdded:
    return "QEvent::ActionAdded";
  case QEvent::ActionChanged:
    return "QEvent::ActionChanged";
  case QEvent::ActionRemoved:
    return "QEvent::ActionRemoved";
  case QEvent::ActivationChange:
    return "QEvent::ActivationChange";
  case QEvent::ApplicationActivate:
    return "QEvent::ApplicationActivate";
  case QEvent::ApplicationDeactivate:
    return "QEvent::ApplicationDeactivate";
  case QEvent::ApplicationFontChange:
    return "QEvent::ApplicationFontChange";
  case QEvent::ApplicationLayoutDirectionChange:
    return "QEvent::ApplicationLayoutDirectionChange";
  case QEvent::ApplicationPaletteChange:
    return "QEvent::ApplicationPaletteChange";
  case QEvent::ApplicationStateChange:
    return "QEvent::ApplicationStateChange";
  case QEvent::ApplicationWindowIconChange:
    return "QEvent::ApplicationWindowIconChange";
  case QEvent::ChildAdded:
    return "QEvent::ChildAdded";
  case QEvent::ChildPolished:
    return "QEvent::ChildPolished";
  case QEvent::ChildRemoved:
    return "QEvent::ChildRemoved";
  case QEvent::Clipboard:
    return "QEvent::Clipboard";
  case QEvent::Close:
    return "QEvent::Close";
  case QEvent::CloseSoftwareInputPanel:
    return "QEvent::CloseSoftwareInputPanel";
  case QEvent::ContentsRectChange:
    return "QEvent::ContentsRectChange";
  case QEvent::ContextMenu:
    return "QEvent::ContextMenu";
  case QEvent::CursorChange:
    return "QEvent::CursorChange";
  case QEvent::DeferredDelete:
    return "QEvent::DeferredDelete";
  case QEvent::DragEnter:
    return "QEvent::DragEnter";
  case QEvent::DragLeave:
    return "QEvent::DragLeave";
  case QEvent::DragMove:
    return "QEvent::DragMove";
  case QEvent::Drop:
    return "QEvent::Drop";
  case QEvent::DynamicPropertyChange:
    return "QEvent::DynamicPropertyChange";
  case QEvent::EnabledChange:
    return "QEvent::EnabledChange";
  case QEvent::Enter:
    return "QEvent::Enter";
  case QEvent::EnterWhatsThisMode:
    return "QEvent::EnterWhatsThisMode";
  case QEvent::Expose:
    return "QEvent::Expose";
  case QEvent::FileOpen:
    return "QEvent::FileOpen";
  case QEvent::FocusIn:
    return "QEvent::FocusIn";
  case QEvent::FocusOut:
    return "QEvent::FocusOut";
  case QEvent::FocusAboutToChange:
    return "QEvent::FocusAboutToChange";
  case QEvent::FontChange:
    return "QEvent::FontChange";
  case QEvent::Gesture:
    return "QEvent::Gesture";
  case QEvent::GestureOverride:
    return "QEvent::GestureOverride";
  case QEvent::GrabKeyboard:
    return "QEvent::GrabKeyboard";
  case QEvent::GrabMouse:
    return "QEvent::GrabMouse";
  case QEvent::GraphicsSceneContextMenu:
    return "QEvent::GraphicsSceneContextMenu";
  case QEvent::GraphicsSceneDragEnter:
    return "QEvent::GraphicsSceneDragEnter";
  case QEvent::GraphicsSceneDragLeave:
    return "QEvent::GraphicsSceneDragLeave";
  case QEvent::GraphicsSceneDragMove:
    return "QEvent::GraphicsSceneDragMove";
  case QEvent::GraphicsSceneDrop:
    return "QEvent::GraphicsSceneDrop";
  case QEvent::GraphicsSceneHelp:
    return "QEvent::GraphicsSceneHelp";
  case QEvent::GraphicsSceneHoverEnter:
    return "QEvent::GraphicsSceneHoverEnter";
  case QEvent::GraphicsSceneHoverLeave:
    return "QEvent::GraphicsSceneHoverLeave";
  case QEvent::GraphicsSceneHoverMove:
    return "QEvent::GraphicsSceneHoverMove";
  case QEvent::GraphicsSceneMouseDoubleClick:
    return "QEvent::GraphicsSceneMouseDoubleClick";
  case QEvent::GraphicsSceneMouseMove:
    return "QEvent::GraphicsSceneMouseMove";
  case QEvent::GraphicsSceneMousePress:
    return "QEvent::GraphicsSceneMousePress";
  case QEvent::GraphicsSceneMouseRelease:
    return "QEvent::GraphicsSceneMouseRelease";
  case QEvent::GraphicsSceneMove:
    return "QEvent::GraphicsSceneMove";
  case QEvent::GraphicsSceneResize:
    return "QEvent::GraphicsSceneResize";
  case QEvent::GraphicsSceneWheel:
    return "QEvent::GraphicsSceneWheel";
  case QEvent::Hide:
    return "QEvent::Hide";
  case QEvent::HideToParent:
    return "QEvent::HideToParent";
  case QEvent::HoverEnter:
    return "QEvent::HoverEnter";
  case QEvent::HoverLeave:
    return "QEvent::HoverLeave";
  case QEvent::HoverMove:
    return "QEvent::HoverMove";
  case QEvent::IconDrag:
    return "QEvent::IconDrag";
  case QEvent::IconTextChange:
    return "QEvent::IconTextChange";
  case QEvent::InputMethod:
    return "QEvent::InputMethod";
  case QEvent::InputMethodQuery:
    return "QEvent::InputMethodQuery";
  case QEvent::KeyboardLayoutChange:
    return "QEvent::KeyboardLayoutChange";
  case QEvent::KeyPress:
    return "QEvent::KeyPress";
  case QEvent::KeyRelease:
    return "QEvent::KeyRelease";
  case QEvent::LanguageChange:
    return "QEvent::LanguageChange";
  case QEvent::LayoutDirectionChange:
    return "QEvent::LayoutDirectionChange";
  case QEvent::LayoutRequest:
    return "QEvent::LayoutRequest";
  case QEvent::Leave:
    return "QEvent::Leave";
  case QEvent::LeaveWhatsThisMode:
    return "QEvent::LeaveWhatsThisMode";
  case QEvent::LocaleChange:
    return "QEvent::LocaleChange";
  case QEvent::NonClientAreaMouseButtonDblClick:
    return "QEvent::NonClientAreaMouseButtonDblClick";
  case QEvent::NonClientAreaMouseButtonPress:
    return "QEvent::NonClientAreaMouseButtonPress";
  case QEvent::NonClientAreaMouseButtonRelease:
    return "QEvent::NonClientAreaMouseButtonRelease";
  case QEvent::NonClientAreaMouseMove:
    return "QEvent::NonClientAreaMouseMove";
  case QEvent::MacSizeChange:
    return "QEvent::MacSizeChange";
  case QEvent::MetaCall:
    return "QEvent::MetaCall";
  case QEvent::ModifiedChange:
    return "QEvent::ModifiedChange";
  case QEvent::MouseButtonDblClick:
    return "QEvent::MouseButtonDblClick";
  case QEvent::MouseButtonPress:
    return "QEvent::MouseButtonPress";
  case QEvent::MouseButtonRelease:
    return "QEvent::MouseButtonRelease";
  case QEvent::MouseMove:
    return "QEvent::MouseMove";
  case QEvent::MouseTrackingChange:
    return "QEvent::MouseTrackingChange";
  case QEvent::Move:
    return "QEvent::Move";
  case QEvent::NativeGesture:
    return "QEvent::NativeGesture";
  case QEvent::OrientationChange:
    return "QEvent::OrientationChange";
  case QEvent::Paint:
    return "QEvent::Paint";
  case QEvent::PaletteChange:
    return "QEvent::PaletteChange";
  case QEvent::ParentAboutToChange:
    return "QEvent::ParentAboutToChange";
  case QEvent::ParentChange:
    return "QEvent::ParentChange";
  case QEvent::PlatformPanel:
    return "QEvent::PlatformPanel";
  case QEvent::PlatformSurface:
    return "QEvent::PlatformSurface";
  case QEvent::Polish:
    return "QEvent::Polish";
  case QEvent::PolishRequest:
    return "QEvent::PolishRequest";
  case QEvent::QueryWhatsThis:
    return "QEvent::QueryWhatsThis";
  case QEvent::ReadOnlyChange:
    return "QEvent::ReadOnlyChange";
  case QEvent::RequestSoftwareInputPanel:
    return "QEvent::RequestSoftwareInputPanel";
  case QEvent::Resize:
    return "QEvent::Resize";
  case QEvent::ScrollPrepare:
    return "QEvent::ScrollPrepare";
  case QEvent::Scroll:
    return "QEvent::Scroll";
  case QEvent::Shortcut:
    return "QEvent::Shortcut";
  case QEvent::ShortcutOverride:
    return "QEvent::ShortcutOverride";
  case QEvent::Show:
    return "QEvent::Show";
  case QEvent::ShowToParent:
    return "QEvent::ShowToParent";
  case QEvent::SockAct:
    return "QEvent::SockAct";
  case QEvent::StateMachineSignal:
    return "QEvent::StateMachineSignal";
  case QEvent::StateMachineWrapped:
    return "QEvent::StateMachineWrapped";
  case QEvent::StatusTip:
    return "QEvent::StatusTip";
  case QEvent::StyleChange:
    return "QEvent::StyleChange";
  case QEvent::TabletMove:
    return "QEvent::TabletMove";
  case QEvent::TabletPress:
    return "QEvent::TabletPress";
  case QEvent::TabletRelease:
    return "QEvent::TabletRelease";
  case QEvent::TabletEnterProximity:
    return "QEvent::TabletEnterProximity";
  case QEvent::TabletLeaveProximity:
    return "QEvent::TabletLeaveProximity";
  case QEvent::TabletTrackingChange:
    return "QEvent::TabletTrackingChange";
  case QEvent::ThreadChange:
    return "QEvent::ThreadChange";
  case QEvent::Timer:
    return "QEvent::Timer";
  case QEvent::ToolBarChange:
    return "QEvent::ToolBarChange";
  case QEvent::ToolTip:
    return "QEvent::ToolTip";
  case QEvent::ToolTipChange:
    return "QEvent::ToolTipChange";
  case QEvent::TouchBegin:
    return "QEvent::TouchBegin";
  case QEvent::TouchCancel:
    return "QEvent::TouchCancel";
  case QEvent::TouchEnd:
    return "QEvent::TouchEnd";
  case QEvent::TouchUpdate:
    return "QEvent::TouchUpdate";
  case QEvent::UngrabKeyboard:
    return "QEvent::UngrabKeyboard";
  case QEvent::UngrabMouse:
    return "QEvent::UngrabMouse";
  case QEvent::UpdateLater:
    return "QEvent::UpdateLater";
  case QEvent::UpdateRequest:
    return "QEvent::UpdateRequest";
  case QEvent::WhatsThis:
    return "QEvent::WhatsThis";
  case QEvent::WhatsThisClicked:
    return "QEvent::WhatsThisClicked";
  case QEvent::Wheel:
    return "QEvent::Wheel";
  case QEvent::WinEventAct:
    return "QEvent::WinEventAct";
  case QEvent::WindowActivate:
    return "QEvent::WindowActivate";
  case QEvent::WindowBlocked:
    return "QEvent::WindowBlocked";
  case QEvent::WindowDeactivate:
    return "QEvent::WindowDeactivate";
  case QEvent::WindowIconChange:
    return "QEvent::WindowIconChange";
  case QEvent::WindowStateChange:
    return "QEvent::WindowStateChange";
  case QEvent::WindowTitleChange:
    return "QEvent::WindowTitleChange";
  case QEvent::WindowUnblocked:
    return "QEvent::WindowUnblocked";
  case QEvent::WinIdChange:
    return "QEvent::WinIdChange";
  case QEvent::ZOrderChange:
    return "QEvent::ZOrderChange";
  case QEvent::Create:
    return "QEvent::Create";
  case QEvent::Destroy:
    return "QEvent::Destroy";
  case QEvent::Quit:
    return "QEvent::Quit";
  case QEvent::Speech:
    return "QEvent::Speech";
  case QEvent::DragResponse:
    return "QEvent::DragResponse";
  case QEvent::ShowWindowRequest:
    return "QEvent::ShowWindowRequest";
  case QEvent::EmbeddingControl:
    return "QEvent::EmbeddingControl";
  case QEvent::ActivateControl:
    return "QEvent::ActivateControl";
  case QEvent::DeactivateControl:
    return "QEvent::DeactivateControl";
  case QEvent::Style:
    return "QEvent::Style";
  case QEvent::OkRequest:
    return "QEvent::OkRequest";
  case QEvent::HelpRequest:
    return "QEvent::HelpRequest";
  case QEvent::AcceptDropsChange:
    return "QEvent::AcceptDropsChange";
  case QEvent::ZeroTimerEvent:
    return "QEvent::ZeroTimerEvent";
  case QEvent::MacGLWindowChange:
    return "QEvent::MacGLWindowChange";
  case QEvent::FutureCallOut:
    return "QEvent::FutureCallOut";
  case QEvent::NetworkReplyUpdated:
    return "QEvent::NetworkReplyUpdated";
  case QEvent::MacGLClearDrawable:
    return "QEvent::MacGLClearDrawable";
  case QEvent::ThemeChange:
    return "QEvent::ThemeChange";
  case QEvent::SockClose:
    return "QEvent::SockClose";
  case QEvent::StyleAnimationUpdate:
    return "QEvent::StyleAnimationUpdate";
  case QEvent::WindowChangeInternal:
    return "QEvent::WindowChangeInternal";
  case QEvent::ScreenChangeInternal:
    return "QEvent::ScreenChangeInternal";
  case QEvent::Pointer:
    return "QEvent::Pointer";
  case QEvent::User:
    return "QEvent::User";
  case QEvent::MaxUser:
    return "QEvent::MaxUser";
  default:
    return "Unknown";
  }
}

} // namespace ecn_baxter::utils::qt

#endif