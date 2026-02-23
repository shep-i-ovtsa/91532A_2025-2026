#include "graphics.hpp"
#include "ratatroller.hpp"

using namespace rat;

void screen::push(std::unique_ptr<screen> s) {
    navigator.push_screen(std::move(s));
}
void screen::pop() {
    navigator.pop_screen();   
}
pros::Controller& screen::get_controller(){
    return navigator.get_controller();
}
void screen::display(int line, int column, const std::string& content) {
    if(line < 0|| column< 0 || column >=SCREEN_WIDTH){
        return;
    }

    display_request r;
    r.type = request_type::PRINT;
    r.line = line;
    r.column = column;
    r.content = content;
    request_queue.push_back(r);
}
void screen::clear_line(int line) {
    if(line < 0)
        return;
    display_request r;
    r.type = request_type::CLEAR_LINE;
    r.line = line;

    request_queue.push_back(r);
}

void screen::clear_screen() {
    display_request r;
    r.type = request_type::CLEAR_SCREEN;
    request_queue.push_back(r);
}

void screen::display_screen() {
    tick();
    process_queue();
    render_screen();
}

void screen::process_queue() {
    for(const auto& r : request_queue) {
        switch(r.type) {
            case request_type::PRINT:
                write_to_canvas(r.line, r.column, r.content);
                break;
            case request_type::CLEAR_LINE:
                if(r.line >= 0 && r.line < canvas.size())
                    canvas[r.line].fill(' ');
                break;
            case request_type::CLEAR_SCREEN:
                for(auto& row : canvas)
                    row.fill(' ');
                break;
        }
    }

    request_queue.clear();
}

void screen::write_to_canvas(int line, int column, const std::string& content) {

    while(line >= static_cast<int>(canvas.size())) {
        std::array<char, SCREEN_WIDTH> row;
        row.fill(' ');
        canvas.push_back(row);
    }
    for(size_t i = 0; i < content.size(); ++i) {
        int col = column + i;
        if(col >= SCREEN_WIDTH)
            break;
        canvas[line][col] = content[i];
    }
}

void screen::render_screen() {
    auto controller = get_controller();
    for(int row = 0; row < SCREEN_HEIGHT; ++row) {
        int canvas_row = viewport_top + row;
        std::string output(SCREEN_WIDTH, ' ');
        if(canvas_row < canvas.size()) {
            output.assign(canvas[canvas_row].begin(),canvas[canvas_row].end());
        }
        controller.clear_line(row);
        controller.print(row, 0, "%s", output.c_str());
        pros::delay(50);
    }
}