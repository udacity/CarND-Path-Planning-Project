#ifndef STATE_H
#define STATE_H

class State {
  public:
    static State& Instance(){
      static State s;
      return s;
    }
    unsigned int lane;

  private:
    State() {
      lane = 1;
    } 
    ~State() {}

    State(State const&); 
    State& operator= (State const&); 
};

#endif
