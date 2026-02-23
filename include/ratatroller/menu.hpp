#ifndef MENU_HPP
#define MENU_HPP

#include "graphics.hpp"
#include <vector>
#include <functional>
#include <string>

struct menu_option {
public:
    explicit menu_option(const std::string& id)
        : id(id) {}

    void set_behavior_select(std::function<void()> fn) {
        on_select = std::move(fn);
    }

    void set_behavior_hover(std::function<void()> fn) {
        on_hover = std::move(fn);
    }

    void set_text(const std::string& t) {
        text = t;
    }

    const std::string& get_id() const {
        return id;
    }

    void select() {
        if (on_select) on_select();
    }

    void hover() {
        if (!is_hovered && on_hover)
            on_hover();
        is_hovered = true;
    }

    void unhover() {
        is_hovered = false;
    }

    bool hovered() const {
        return is_hovered;
    }

    const std::string& get_text() const {
        return text;
    }

private:
    std::function<void()> on_select{};
    std::function<void()> on_hover{};
    bool is_hovered = false;

    std::string id;
    std::string text;
};

class menu_panel : public rat::screen {
public:
    explicit menu_panel(rat::ratatroller& nav)
        : screen(nav) {}

    void handle_action(rat::action a) override;
    void tick() override;  

    menu_option& add_option(const std::string& id,
                            const std::string& text);

    void set_text_by_id(const std::string& id,
                        const std::string& text);

private:
    void check_visible();

    std::vector<menu_option> options;
    std::size_t selected_index = 0;
};

#endif