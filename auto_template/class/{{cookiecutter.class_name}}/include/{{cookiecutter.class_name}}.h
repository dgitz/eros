/*! \file {{cookiecutter.class_name}}.h
 */
#ifndef {{cookiecutter.class_name}}_H
#define {{cookiecutter.class_name}}_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// ROS Services
// ROS Actions
// Project Includes
using namespace eros;
/*! \class {{cookiecutter.class_name}}
    \brief {{cookiecutter.class_name}}
    A description of your new Class.
*/
class {{cookiecutter.class_name}} {
    public:
    // Constructors & Desostructors
    {{cookiecutter.class_name}}();
    virtual ~{{cookiecutter.class_name}}() {
    }
    // Constants

    // Enums
    enum class Enum1 {
        UNKNOWN = 0,        /*!< Uninitialized value. */
        END_OF_LIST = 1     /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert {{cookiecutter.class_name}}::Enum1 to human readable string
    /*!
      \param v {{cookiecutter.class_name}}::Enum1 type
      \return The converted string.
    */
    static std::string Enum1String({{cookiecutter.class_name}}::Enum1 v) {
        switch (v) {
            case {{cookiecutter.class_name}}::Enum1::UNKNOWN: return "UNKNOWN"; break;
            default: return Enum1String({{cookiecutter.class_name}}::Enum1::UNKNOWN); break;
        }
    }

    // Structs

    // Initializing & Reset Functions

    // Update Functions

    // Attribute Functions

    // Message Functions

    // Support Functions

    // Printing Functions

    // Finish Functions
    bool finish();
    private:
};

#endif // {{cookiecutter.class_name}}_H