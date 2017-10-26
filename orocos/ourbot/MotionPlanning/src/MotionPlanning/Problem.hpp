class ProblemInterface {
    public:
        virtual int n_obstacles() = 0;
        virtual void reset() = 0;
        virtual void resetTime() = 0;
        virtual void recover() = 0;
        virtual void getCoefficients(std::vector<double>& coeff) = 0;
};

template <class problemtype> class Problem : public ProblemInterface {
    protected:
        problemtype* _problem;

    public:

        Problem(problemtype* problem) {
            this->_problem = problem;
        }

        int n_obstacles() {
            return this->_problem->n_obs;
        }

        void reset() {
            this->_problem->reset();
        }

        void resetTime() {
            this->_problem->resetTime();
        }

        void recover() {
            this->_problem->recover();
        }

        void getCoefficients(std::vector<double>& coeff) {
            this->_problem->getCoefficients(coeff);
        }

        problemtype* get() {
            return this->_problem;
        }

};

template <class problemtype> class ADMMProblem : public Problem<problemtype> {

    public:

        ADMMProblem(problemtype* problem) : Problem<problemtype>(problem) {};

        int n_shared() {
            return this->_problem->n_shared;
        }

        int iteration() {
            return this->_problem->getIteration();
        }

        void stepBack() {
            this->_problem->stepBack();
        }
};
