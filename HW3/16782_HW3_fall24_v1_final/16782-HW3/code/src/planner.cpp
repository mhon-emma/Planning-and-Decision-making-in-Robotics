#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <climits> // Added for INT_MAX
#include <chrono>  // Added for measuring runtime

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#ifndef ENVS_DIR
#define ENVS_DIR "../envs"
#endif
class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

int nodes_expanded = 0;

// Extra Credit: turn on heuristic
bool use_heuristic = true; 

class GroundedCondition
{
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(const string &predicate, const list<string> &arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;
        for (const string& l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;
        for (const string& l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        temp += this->predicate;
        temp += "(";
        for (const string& l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(const string &pred, const list<string>& args, const bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (const string& ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (const string& l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(const string &name, const list<string>& args,
           const unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
           const unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (const string& l : args)
        {
            this->args.push_back(l);
        }
        for (const Condition& pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (const Condition& pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (const Condition& precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (const Condition& effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->get_name();
        temp += "(";
        for (const string& l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(const GroundedCondition& gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(const GroundedCondition& gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(const GroundedCondition& gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(const GroundedCondition& gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(const string& symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(const list<string>& symbols)
    {
        for (const string& l : symbols)
            this->symbols.insert(l);
    }
    void add_action(const Action& action)
    {
        this->actions.insert(action);
    }

    Action get_action(const string& name) const {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (const string& s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (const GroundedCondition& s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (const GroundedCondition& g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (const Action& g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }

    // add getters
    auto get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    auto get_goal_conditions() const
    {
        return this->goal_conditions;
    }
    auto get_actions() const
    {
        return this->actions;
    }
};

class GroundedAction
{
    string name;
    list<string> arg_values;

public:
    GroundedAction(const string &name, const list<string>& arg_values)
    {
        this->name = name;
        for (const string& ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->name;
        temp += "(";
        for (const string& l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line.empty())
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        string predicate = iter->str();
                        iter++;
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    action_name = iter->str();
                    iter++;
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

//extra credit 
int heuristic(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& state,
              const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& goal_conditions) {
    int h = 0;
    for (const auto& goal : goal_conditions) 
    {
        if (state.find(goal) == state.end()) 
        {
            h++;
        }
    }
    return h;
}

//add goal and heuristic
class Node
{
public:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state;
    list<GroundedAction> plan;
    int g; 
    int h;
    // add to compare nodes
    bool operator>(const Node& other) const {return (g + h) > (other.g + other.h);}
};

list<GroundedAction> planner(Env* env)
{
    Node initial_node;
    initial_node.state = env->get_initial_conditions();
    initial_node.plan = list<GroundedAction>();
    initial_node.g = 0; 
    if (use_heuristic) {initial_node.h = heuristic(initial_node.state, env->get_goal_conditions());} else {initial_node.h = 0;} 

    auto nodeComparator = [](const Node& lhs, const Node& rhs) {return (lhs.g + lhs.h) > (rhs.g + rhs.h);};
    priority_queue<Node, vector<Node>, decltype(nodeComparator)> frontier(nodeComparator); //use for heuristiC
    frontier.push(initial_node);

    unordered_set<string> explored_states;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions = env->get_goal_conditions();

    // A* Search loop
    while (!frontier.empty())
    {
        Node current_node = frontier.top();
        frontier.pop();

        nodes_expanded++; 

        bool goal_satisfied = true;
        for (const GroundedCondition& goal_cond : goal_conditions)
        {
            if (goal_cond.get_truth())
            {
                if (current_node.state.find(goal_cond) == current_node.state.end())
                {
                    goal_satisfied = false;
                    break;
                }
            }
            else
            {
                GroundedCondition positive_goal_cond(goal_cond.get_predicate(), goal_cond.get_arg_values(), true);
                if (current_node.state.find(positive_goal_cond) != current_node.state.end())
                {
                    goal_satisfied = false;
                    break;
                }
            }
        }

        if (goal_satisfied)
        {return current_node.plan;}

        string state_str;
        vector<string> conditions_str;
        for (const GroundedCondition& cond : current_node.state)
        {conditions_str.push_back(cond.toString());}
        sort(conditions_str.begin(), conditions_str.end());
        for (const string& cond_str : conditions_str)
        {state_str += cond_str + ";";}

        if (explored_states.find(state_str) != explored_states.end())
        {continue;}

        explored_states.insert(state_str);

        for (const Action& action : env->get_actions())
        {
            // all possibility
            list<string> action_params = action.get_args();
            int num_params = action_params.size();
            vector<string> param_names(action_params.begin(), action_params.end());

            // all combinations of parameter assignments
            unordered_set<string> symbols_set = env->get_symbols();
            vector<string> symbols(symbols_set.begin(), symbols_set.end());

            //recursive
            vector<string> param_values(num_params);
            function<void(int)> generate_bindings = [&](int index) 
            {
                if (index == num_params) 
                {
                    unordered_map<string, string> binding;
                    for (int i = 0; i < num_params; ++i) {binding[param_names[i]] = param_values[i];}

                    bool applicable = true;
                    for (const Condition& precond : action.get_preconditions()) 
                    {
                        string predicate = precond.get_predicate();
                        list<string> args;
                        for (const string& arg : precond.get_args()) 
                        {
                            if (binding.find(arg) != binding.end()){args.push_back(binding[arg]);} else {args.push_back(arg);}
                        }

                        GroundedCondition grounded_precond(predicate, args, precond.get_truth());

                        if (precond.get_truth()) {
                            if (current_node.state.find(grounded_precond) == current_node.state.end()) 
                            {
                                applicable = false;
                                break;
                            }
                        } else {
                            GroundedCondition positive_precond(predicate, args, true);
                            if (current_node.state.find(positive_precond) != current_node.state.end()) 
                            {
                                applicable = false;
                                break;
                            }
                        }
                    }

                    if (applicable) { 
                        Node child_node;
                        child_node.state = current_node.state;
                        child_node.plan = current_node.plan;
                        child_node.plan.push_back(GroundedAction(action.get_name(), list<string>(param_values.begin(), param_values.end())));
                        child_node.g = current_node.g + 1; //just 1 cost per action
                        //empty delete list heuristic 
                        if (use_heuristic) {child_node.h = heuristic(child_node.state, goal_conditions);} else {child_node.h = 0;}
                        for (const Condition& effect : action.get_effects()) 
                        {
                            string predicate = effect.get_predicate();
                            list<string> args;
                            for (const string& arg : effect.get_args()) 
                            {
                                if (binding.find(arg) != binding.end()) {args.push_back(binding[arg]);} else {args.push_back(arg);}
                            }

                            GroundedCondition grounded_effect(predicate, args, effect.get_truth());

                            if (effect.get_truth()) 
                            {child_node.state.insert(grounded_effect);} 
                            else 
                            {
                                GroundedCondition positive_effect(predicate, args, true);
                                child_node.state.erase(positive_effect);
                            }
                        }
                        frontier.push(child_node);
                    }

                    return;
                } else 
                {
                    for (const string& symbol : symbols) 
                    {
                        param_values[index] = symbol;
                        generate_bindings(index + 1);
                    }
                }
            };
            generate_bindings(0);
        }
    }

    cout << "No plan found." << endl;
    return list<GroundedAction>();
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* env_file = static_cast<char *>("example.txt");
    if (argc > 1)
        env_file = argv[1];
    std::string envsDirPath = ENVS_DIR;
    char* filename = new char[envsDirPath.length() + strlen(env_file) + 2];
    strcpy(filename, envsDirPath.c_str());
    strcat(filename, "/");
    strcat(filename, env_file);

    cout << "Environment: " << filename << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    // Measure runtime
    auto start_time = chrono::high_resolution_clock::now();

    list<GroundedAction> actions = planner(env);

    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();

    cout << "\nPlan: " << endl;
    for (const GroundedAction& gac : actions)
    {
        cout << gac << endl;
    }
    
    cout << "Heuristic:" << use_heuristic <<endl;
    cout << "Runtime: " << duration << " ms" << endl;
    cout << "Nodes Expanded: " << nodes_expanded << endl;

    delete env;
    return 0;
}
