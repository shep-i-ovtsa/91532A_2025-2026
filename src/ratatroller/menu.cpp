#include "menu.hpp"

menu_option& menu_panel::add_option(const std::string& id,
                                    const std::string& text)
{
    options.emplace_back(id);
    options.back().set_text(text);

    if (options.size() == 1) {
        selected_index = 0;
        options[0].hover();
    }

    return options.back();
}

void menu_panel::handle_action(rat::action a)
{
    if (options.empty()) return;

    switch (a) {

        case rat::action::NAV_UP:
            if (selected_index > 0) {
                options[selected_index].unhover();
                selected_index--;
                options[selected_index].hover();
                check_visible();
            }
            break;

        case rat::action::NAV_DOWN:
            if (selected_index + 1 < options.size()) {
                options[selected_index].unhover();
                selected_index++;
                options[selected_index].hover();
                check_visible();
            }
            break;

        case rat::action::SELECT:
            options[selected_index].select();
            break;

        case rat::action::BACK:
            pop();
            break;

        default:
            break;
    }
}

void menu_panel::check_visible()
{
    // keep selection inside viewport
    if (selected_index < viewport_top) {
        scroll_up();
    }
    else if (selected_index >= viewport_top + SCREEN_HEIGHT) {
        scroll_down();
    }
}

void menu_panel::tick()
{
    clear_screen();

    for (std::size_t i = 0; i < options.size(); ++i) {

        std::string prefix = (i == selected_index) ? "> " : "  ";
        display(static_cast<int>(i), 0,
                prefix + options[i].get_text());
    }
}

void menu_panel::set_text_by_id(const std::string& id,
                                const std::string& text)
{
    for (auto& opt : options) {
        if (opt.get_id() == id) {
            opt.set_text(text);
            break;
        }
    }
}