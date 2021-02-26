pragma Singleton
import QtQuick 2.9

QtObject {
    readonly property int total_screen_width: main_screen_width + side_bar_width
    readonly property int total_screen_height: main_screen_height

    readonly property int main_screen_width: 1150
    readonly property int main_screen_height: 900

    readonly property int side_bar_width: 250
    readonly property int side_bar_button_height: 100

    readonly property int default_panel_width: 350
    readonly property int default_motor_control_panel_width: 220
    readonly property int default_measurement_panel_wdith: 240
    readonly property int default_small_text_rect_height: 50
    readonly property int measurement_reading_box_width: 100

    readonly property int default_small_edge_radius: 8
    readonly property int default_large_edge_radius: 16

    readonly property color dark_purple: "#1c163c"
    readonly property color muted_purple: "#2d2d57"
    readonly property color medium_purple: "purple"
    readonly property color light_purple: "#d0caf9"

    readonly property color dark_green: "#011809"
    readonly property color light_green: "#0ca543"
    readonly property color light_red: "#fb8784"
    readonly property color dark_red: "#5a0705"

    readonly property color noncrit_warn_yellow: "orange"
    readonly property color crit_warn_yellow: "yellow"

    readonly property color background_color: dark_purple
    readonly property color subsection_background_color: muted_purple
    readonly property color primary_font_color: "white"

    readonly property int noncritical_error_banner_height: small_text_size * 2 + component_margin * 2
    readonly property int critical_error_banner_height: medium_text_size * 2

    readonly property int screen_margin: 32
    readonly property int component_margin: 8

    readonly property int small_text_size: 20
    readonly property int medium_text_size: 32
    readonly property int large_text_size: 42

    readonly property double default_disabled_opacity: 0.2

}
