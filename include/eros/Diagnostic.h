/*! \file Diagnostic.h
 */
#ifndef EROSDIAGNOSTIC_H
#define EROSDIAGNOSTIC_H
#include <eros/eROS_Definitions.h>

#include <algorithm>
#include <string>
#include <vector>
/*! \class Diagnostic
    \brief Diagnostic class
    Diagnostic class used to create and update diagnostic information.
*/
class Diagnostic
{
   public:
    enum class DiagnosticType {
        UNKNOWN = 0,          /*!< Uninitialized value. */
        UNKNOWN_TYPE = 1,     /*!< A Type that is not defined yet.*/
        SOFTWARE = 2,         /*!< Diagnostic related to Software. */
        COMMUNICATIONS = 3,   /*!< Diagnostic related to Communications. */
        SENSORS = 4,          /*!< Diagnostic related to Sensors. */
        ACTUATORS = 5,        /*!< Diagnostic related to Actuators. */
        DATA_STORAGE = 6,     /*!< Diagnostic related to Data Storage. */
        REMOTE_CONTROL = 7,   /*!< Diagnostic related to Remote Control. */
        TARGETING = 8,        /*!< Diagnostic related to Targeting. */
        POSE = 9,             /*!< Diagnostic related to Pose. */
        TIMING = 10,          /*!< *Diagnostic related to Timing. */
        SYSTEM_RESOURCE = 11, /*!< Diagnostic related to System Resource Usage. */
        END_OF_LIST = 12      /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert Diagnostic::DiagnosticType to human readable string
    /*!
      \param v Diagnostic::DiagnosticType type
      \return The converted string.
    */
    static std::string DiagnosticTypeString(Diagnostic::DiagnosticType v) {
        switch (v) {
            case Diagnostic::DiagnosticType::UNKNOWN: return "UNKNOWN"; break;
            case Diagnostic::DiagnosticType::UNKNOWN_TYPE: return "UNKNOWN_TYPE"; break;
            case Diagnostic::DiagnosticType::SOFTWARE: return "SOFTWARE"; break;
            case Diagnostic::DiagnosticType::COMMUNICATIONS: return "COMMUNICATIONS"; break;
            case Diagnostic::DiagnosticType::SENSORS: return "SENSORS"; break;
            case Diagnostic::DiagnosticType::ACTUATORS: return "ACTUATORS"; break;
            case Diagnostic::DiagnosticType::DATA_STORAGE: return "DATA_STORAGE"; break;
            case Diagnostic::DiagnosticType::REMOTE_CONTROL: return "REMOTE_CONTROL"; break;
            case Diagnostic::DiagnosticType::TARGETING: return "TARGETING"; break;
            case Diagnostic::DiagnosticType::POSE: return "POSE"; break;
            case Diagnostic::DiagnosticType::TIMING: return "TIMING"; break;
            case Diagnostic::DiagnosticType::SYSTEM_RESOURCE: return "SYSTEM_RESOURCE"; break;
            default: return DiagnosticTypeString(Diagnostic::DiagnosticType::UNKNOWN); break;
        }
    }
    enum class Message {
        UNKNOWN = 0,              /*!< Uninitialized value. */
        NOERROR = 1,              /*!< No Error.*/
        UNKNOWN_ERROR = 2,        /*!< Unknown Error Occurred/Error not defined. */
        INITIALIZING = 3,         /*!< Initializing. */
        INITIALIZING_ERROR = 4,   /*!< Error occured during initialization. */
        DROPPING_PACKETS = 5,     /*!< Missing/Dropping Messages. */
        MISSING_HEARTBEATS = 6,   /*!< Missing Heartbeat Messages. */
        DEVICE_NOT_AVAILABLE = 7, /*!< Device is not currently Available. */
        TEMPERATURE_HIGH = 8,     /*!< Temperature too High. */
        TEMPERATURE_LOW = 9,      /*!< Diagnostic Failed. */
        RESOURCE_LEAK = 10,       /*!< Resource Leakage occurred. */
        HIGH_RESOURCE_USAGE = 11, /*!< High Resource Usage. */
        DIAGNOSTIC_FAILED = 12,   /*! < Diagnostic Failed.*/
        END_OF_LIST = 13          /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert Diagnostic::Message to human readable string
    /*!
      \param v Diagnostic::Message type
      \return The converted string.
    */
    static std::string DiagnosticMessageString(Diagnostic::Message v) {
        switch (v) {
            case Diagnostic::Message::UNKNOWN: return "UNKNOWN"; break;
            case Diagnostic::Message::NOERROR: return "NOERROR"; break;
            case Diagnostic::Message::UNKNOWN_ERROR: return "UNKNOWN_ERROR"; break;
            case Diagnostic::Message::INITIALIZING: return "INITIALIZING"; break;
            case Diagnostic::Message::INITIALIZING_ERROR: return "INITIALIZING_ERROR"; break;
            case Diagnostic::Message::DROPPING_PACKETS: return "DROPPING_PACKETS"; break;
            case Diagnostic::Message::MISSING_HEARTBEATS: return "MISSING_HEARTBEATS"; break;
            case Diagnostic::Message::DEVICE_NOT_AVAILABLE: return "DEVICE_NOT_AVAILABLE"; break;
            case Diagnostic::Message::TEMPERATURE_HIGH: return "TEMPERATURE_HIGH"; break;
            case Diagnostic::Message::TEMPERATURE_LOW: return "TEMPERATURE_LOW"; break;
            case Diagnostic::Message::RESOURCE_LEAK: return "RESOURCE_LEAK"; break;
            case Diagnostic::Message::HIGH_RESOURCE_USAGE: return "HIGH_RESOURCE_USAGE"; break;
            case Diagnostic::Message::DIAGNOSTIC_FAILED: return "DIAGNOSTIC_FAILED"; break;
            default: return DiagnosticMessageString(Diagnostic::Message::UNKNOWN); break;
        }
    }
    /*! \struct DiagnosticDefinition
    \brief Contains the definition for the DiagnosticDefinition.

    */
    struct DiagnosticDefinition {
        std::string device_name;
        std::string node_name;
        System::MainSystem system;
        System::SubSystem subsystem;
        System::Component component;
        Diagnostic::DiagnosticType type;
        Diagnostic::Message message;
        Level::Type level;
        std::string description;
    };
    Diagnostic() : initialized(false) {
    }
    virtual ~Diagnostic() {
    }

    //! Generate human readable string of all enabled diagnostics.
    /*!
        \return Human readable string of all enabled diagnostics.
    */
    std::string pretty() {
        return pretty("", diagnostics);
    }

    //! Generate human readable string of a vector of diagnostics
    /*!
        \param pre A string to be pre-appended to the human readable string.
        \param diagnostics A vector of diagnostics.
        \return Human readable string of diagnostics.
    */
    static std::string pretty(std::string pre, std::vector<DiagnosticDefinition> diagnostics) {
        std::string str = "";
        if (diagnostics.size() == 0) {
            str = pre + "NO DIAGNOSTICS DEFINED YET.";
            return str;
        }
        for (std::size_t i = 0; i < diagnostics.size(); ++i) {
            str += "Diag: \n" + pretty("\t", diagnostics.at(i)) + "\n";
        }
        return str;
    }

    //! Generate human readable string of a diagnostic
    /*!
        \param pre A string to be pre-appended to the human readable string.
        \param diag A diagnostic.
        \return Human readable string of diagnostic.
    */
    static std::string pretty(std::string pre, DiagnosticDefinition diag) {
        std::string str = "";
        str += pre + "Level: " + Level::LevelString(diag.level) + "\n";
        str += pre + "Device: " + diag.device_name + "\n";
        str += pre + "Node: " + diag.node_name + "\n";
        str += pre + "System: " + System::MainSystemString(diag.system) + "\n";
        str += pre + "Subsystem: " + System::SubSystemString(diag.subsystem) + "\n";
        str += pre + "Component: " + System::ComponentString(diag.component) + "\n";
        str += pre + "Diagnostic Type: " + Diagnostic::DiagnosticTypeString(diag.type) + "\n";
        str += pre + "Message: " + Diagnostic::DiagnosticMessageString(diag.message) + "\n";
        str += pre + "Desc: " + diag.description;
        return str;
    }

    //! Initialize Root Diagnostic.  This should be called once per instance of Diagnostic
    /*!
      \param device_name The name of the Device.  Can be hostname or any name that makes sense for
      the device.
      \param node_name The name of the node running that has an instance of this class.
      \param system The System identifier, uses enum: System::MainSystem
      \param subsystem The SubSystem identifier, uses enum: System::SubSystem
      \param component The Component identifier, uses enum: System::Component
    */
    void initialize(std::string device_name,
                    std::string node_name,
                    System::MainSystem system,
                    System::SubSystem subsystem,
                    System::Component component) {
        root_diagnostic.device_name = device_name;
        root_diagnostic.node_name = node_name;
        root_diagnostic.system = system;
        root_diagnostic.subsystem = subsystem;
        root_diagnostic.component = component;
        initialized = true;
    }

    //! Enable Diagnostic Types
    /*!
        \param diagnostic_types A vector of Diagnostic Types that should be enabled.
        \return If the diagnostics were enabled or not.
    */
    bool enable_diagnostics(std::vector<Diagnostic::DiagnosticType> diagnostic_types) {
        if (initialized == false) {
            return false;
        }
        std::sort(diagnostic_types.begin(), diagnostic_types.end());
        for (std::size_t i = 0; i < diagnostic_types.size(); ++i) {
            DiagnosticDefinition diag = root_diagnostic;
            diag.type = diagnostic_types.at(i);
            if (diag.type ==
                Diagnostic::DiagnosticType::SYSTEM_RESOURCE)  // This is special, so we don't throw
                                                              // a ton of warn messages when the
                                                              // system launches.
            {
                diag.level = Level::Type::NOTICE;
                diag.message = Diagnostic::Message::INITIALIZING;
                diag.description = "Initializing Resource Monitor.";
            }
            else {
                diag.level = Level::Type::WARN;
                diag.message = Diagnostic::Message::INITIALIZING;
                diag.description = "Initializing Diagnostic.";
            }
            diagnostics.push_back(diag);
        }
        return true;
    }

    DiagnosticDefinition get_root_diagnostic() {
        return root_diagnostic;
    }
    std::vector<DiagnosticDefinition> get_diagnostics() {
        return diagnostics;
    }

    //! Update Diagnostic
    /*!
        \brief Updates an enabled diagnostic with the defined information.
    */
    DiagnosticDefinition update_diagnostic(Diagnostic::DiagnosticType diagnostic_type,
                                           Level::Type level,
                                           Diagnostic::Message message,
                                           std::string description) {
        return update_diagnostic(
            root_diagnostic.device_name, diagnostic_type, level, message, description);
    }

    //! Update Diagnostic
    /*!
        \brief Updates an enabled diagnostic with the provided diagnostic.
    */
    DiagnosticDefinition update_diagnostic(DiagnosticDefinition diag) {
        return update_diagnostic(
            diag.device_name, diag.type, diag.level, diag.message, diag.description);
    }

    //! Update Diagnostic
    /*!
        \brief Updates an enabled diagnostic with the fully defined diagnostic information.
    */
    DiagnosticDefinition update_diagnostic(std::string device_name,
                                           Diagnostic::DiagnosticType diagnostic_type,
                                           Level::Type level,
                                           Diagnostic::Message message,
                                           std::string description) {
        bool devicetype_found = false;
        bool devicename_found = false;
        DiagnosticDefinition diag;
        uint8_t insert_index = -1;
        for (std::size_t i = 0; i < diagnostics.size(); ++i) {
            if (diagnostic_type == diagnostics.at(i).type) {
                devicetype_found = true;
                insert_index = i;
                if (diagnostics.at(i).device_name == device_name) {
                    devicename_found = true;
                    diag = diagnostics.at(i);
                    diag.level = level;
                    diag.message = message;
                    diag.description = description;
                    diagnostics.at(i) = diag;
                }
            }
        }
        if ((devicetype_found == true) and (devicename_found == false)) {
            diag = root_diagnostic;
            diag.type = diagnostic_type;
            diag.device_name = device_name;
            diag.level = level;
            diag.message = message;
            diag.description = description;
            std::vector<DiagnosticDefinition>::iterator it;
            it = diagnostics.begin();
            diagnostics.insert(it + insert_index, diag);
        }
        if (devicetype_found == true) {
            return diag;
        }
        else {
            diag = root_diagnostic;
            diag.type = diagnostic_type;
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::INITIALIZING_ERROR;
            char tempstr[512];
            sprintf(tempstr,
                    "Unsupported Diagnostic Type: %s(%d).  Did you forget to enable it?",
                    DiagnosticTypeString(diagnostic_type).c_str(),
                    (uint8_t)diagnostic_type);
            diag.description = std::string(tempstr);
            return diag;
        }
    }

   private:
    bool initialized;
    DiagnosticDefinition root_diagnostic;
    std::vector<DiagnosticDefinition> diagnostics;
};

#endif  // EROSDIAGNOSTIC_H