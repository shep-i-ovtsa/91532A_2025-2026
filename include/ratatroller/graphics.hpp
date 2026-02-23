#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include "EZ-Template/util.hpp"
#include "keybinds.hpp"
#include <memory>
#include <vector>
#include <array>
#include <string>

namespace rat {

class ratatroller; 

class screen {

protected:
    ratatroller& navigator;    

public:
    static const int SCREEN_WIDTH = 15;
    static const int SCREEN_HEIGHT = 3;
    enum class request_type {
        PRINT,
        CLEAR_LINE,
        CLEAR_SCREEN
    };

    struct display_request {
        request_type type;

        std::string content;
        int line = 0;
        int column = 0;


        int column_start = 0;
        int column_end = SCREEN_WIDTH - 1;
    };

    explicit screen(ratatroller& nav)
        : navigator(nav), viewport_top(0)
    {
        for(int i = 0; i < SCREEN_HEIGHT; ++i) {
            std::array<char, SCREEN_WIDTH> row;
            row.fill(' ');
            canvas.push_back(row);
        }
    }

    virtual ~screen() = default;
    screen(const screen&) = delete;
    screen& operator=(const screen&) = delete;
    screen(screen&&) = delete;
    screen& operator=(screen&&) = delete;

    virtual void handle_action(action a) = 0;
    virtual void display_screen();
    virtual void tick();
    void push(std::unique_ptr<screen> s);
    void pop();
    pros::Controller& get_controller();

    void display(int line, int column, const std::string& content);
    void clear_line(int line);
    void clear_screen();
    void scroll_up();
    void scroll_down();    
    int viewport_top;
protected:
    void process_queue();
    void write_to_canvas(int line, int column, const std::string& content);
    void render_screen();

private:


    std::vector<display_request> request_queue;
    std::vector<std::array<char, SCREEN_WIDTH>> canvas;
};

}

#endif