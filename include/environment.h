#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// This needs to be included before the GL includes, which
// happen in draw.h

// #include "imgui.h"
// #include "imgui_impl_glfw.h"
// #include "imgui_impl_opengl3.h"



#include <vector>
#include <memory>
#include "arena.h"
#include "robot.h"
#include "load.h"
#include "draw.h"

template<class T>
using uvector = std::vector<std::unique_ptr<T> >;

class ContactListener : public b2ContactListener
{
    void BeginContact(b2Contact* c);
    void EndContact(b2Contact* c);
};


class Environment
{
public:
    Environment();
    ~Environment();
    
    void addArena();
    void addRobot(const b2Vec2& position, float angle);
    void addLoad(const b2Vec2& position, unsigned short porters);
    Robot* getNextRobot();
    float run(unsigned long long duration);
    
    bool render;
    bool new_prox = false;
    
private:
    friend class Arena;
    friend class Robot;
    friend class Load;
    
    void destroyLoad(Load* load);
    void resetLoad(unsigned short id);
    
    void sense();
    void commsOut();
    void commsIn();
    void control();
    void act();
    void simStep();
    void renderSetup();
    void renderPredraw();
    void renderStep();
    void renderCleanup();
    void initFitness();
    void updateFitness();
    float finalFitness();
    void drawArena();
    void drawLoads();
    void drawRobots();
    void drawContacts();
    void process_visible(std::unique_ptr<Robot> &robot);


    
    b2World* p_world;
    std::unique_ptr<Arena> p_arena;
    uvector<Robot> p_robots;
    uvector<Load> p_loads;
    std::size_t p_next;
    float p_f_sim;
    float p_fitness;
    GLFWwindow* p_window;
    ContactListener contactlistener_inst;
    
    // Colours
    b2Color black = b2Color(0.0f, 0.0f, 0.0f);
    b2Color red = b2Color(1.0f, 0.0f, 0.0f);
    b2Color green = b2Color(0.0f, 0.7f, 0.0f);
    b2Color orange = b2Color(1.0f, 0.6f, 0.0f);
    b2Color blue = b2Color(0.0f, 0.0f, 1.0f);
    b2Vec2 a_v[4];
    
    uint32_t loop = 0;
    uint32_t physics_per_ctrl;
    std::vector<b2Vec2> sensor_contacts;
    
    b2Vec2 mouse;
    b2Vec2 *centre;
    b2Vec2 drag_origin;
    b2Vec2 start_drag;
    b2Vec2 scroll;
    float zoom;
    bool drag = false;
    
    
    static void cursor_pos_callback(GLFWwindow *w, double xpos, double ypos)
    {
        // We can only pass a class function, not a member function as a callback,
        // so we set the glfw user pointer to the class instance and use this
        // to access the member variables
        auto ptr = static_cast<Environment*>(glfwGetWindowUserPointer(w));
        ptr->mouse.x = xpos;
        ptr->mouse.y = ypos;
    }
    static void mouse_button_callback(GLFWwindow *w, int button, int action, int mods)
    {
        auto ptr = static_cast<Environment*>(glfwGetWindowUserPointer(w));
        if (button == GLFW_MOUSE_BUTTON_RIGHT)
            if (action == GLFW_PRESS)
            {
                ptr->drag = true;
                ptr->start_drag = ptr->mouse;
                ptr->drag_origin = *(ptr->centre);
            }
            if (action == GLFW_RELEASE)
                ptr->drag = false;
    }
    static void scroll_callback(GLFWwindow *w, double xoffset, double yoffset)
    {
        auto ptr = static_cast<Environment*>(glfwGetWindowUserPointer(w));
        ptr->scroll.y = yoffset;
        ptr->zoom *= 1 - 0.01 * yoffset;
        
        
    }
};

#endif
