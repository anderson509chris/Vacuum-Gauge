#include "commandProcessor.h"

// =============================================================================
//  Module-level singleton pointer
//
//  The built-in listCommands / listCommand helpers are plain C functions (not
//  methods) because they are stored as CMDfunction callbacks in the Command
//  table.  They need a way to reach the processor instance.  A static pointer
//  set in the constructor serves that purpose.
//
//  LIMITATION: Only one commandProcessor instance may exist at a time.  If a
//  second instance is constructed it will overwrite CP and the first instance's
//  built-in commands will silently operate on the wrong object.
// =============================================================================

static commandProcessor *CP = NULL;

// =============================================================================
//  Built-in command callbacks (registered automatically in the constructor)
// =============================================================================

/**
 * @brief GCMDS — print every registered command and its help string.
 *
 * Accepts no arguments.  Commands stored with a '?' prefix are listed twice:
 * once with a 'G' prefix and once with an 'S' prefix to show both forms.
 */
void listCommands(void)
{
    if (CP->numArgs > 0) { CP->sendNAK(); return; }
    CP->sendACK(false);
    if (CP->commands == NULL) return;

    CommandList *cl = CP->commands;
    while (cl != NULL)
    {
        for (int i = 0; cl->cmds[i].cmd != NULL; i++)
        {
            if (cl->cmds[i].cmd[0] == '?')
            {
                // Expose as both GCMD and SCMD
                CP->print("G");
                CP->print(&cl->cmds[i].cmd[1]);
                CP->print(",");
                CP->println(cl->cmds[i].help);

                CP->print("S");
                CP->print(&cl->cmds[i].cmd[1]);
                CP->print(",");
                CP->println(cl->cmds[i].help);
            }
            else
            {
                CP->print(cl->cmds[i].cmd);
                CP->print(",");
                CP->println(cl->cmds[i].help);
            }
        }
        cl = (CommandList *)cl->next;
    }
}

/**
 * @brief HELP — print one specific command and its help string.
 *
 * Expects exactly one argument: the command name to look up.
 */
void listCommand(void)
{
    if (CP->numArgs != 1) { CP->sendNAK(); return; }
    CP->sendACK(false);

    // Read the requested command name into cmd so findCommand() can match it.
    CP->rb->getToken(CP->cmd, DELIM);

    Command *c = CP->findCommand();
    if (c == NULL)
    {
        // The requested command name was not found.
        CP->sendNAK(ERR_CMD);
        return;
    }

    if (c->cmd[0] == '?')
    {
        CP->print("G");
        CP->print(&c->cmd[1]);
        CP->print(",");
        CP->println(c->help);

        CP->print("S");
        CP->print(&c->cmd[1]);
        CP->print(",");
        CP->println(c->help);
    }
    else
    {
        CP->print(c->cmd);
        CP->print(",");
        CP->println(c->help);
    }
}

// ---------------------------------------------------------------------------
// Built-in command table and list node (defined at file scope so that the
// constructor can point Command::pointer members at instance variables).
// ---------------------------------------------------------------------------

Command cpCmds[] =
{
    // cmd      type          nargs  pointer  options  help
    {"?ECHO",   CMDbool,      -1,    NULL,    NULL,    "Echo mode, TRUE or FALSE"},
    {"?MUTE",   CMDbool,      -1,    NULL,    NULL,    "Mute mode, TRUE or FALSE"},
    {"?CASE",   CMDbool,      -1,    NULL,    NULL,    "Case sensitive, TRUE or FALSE"},
    {"GCMDS",   CMDfunction,  -1,    (void *)listCommands, NULL, "List all commands and help strings"},
    {"HELP",    CMDfunction,  -1,    (void *)listCommand,  NULL, "List requested command and its help string"},
    {NULL}  // sentinel — marks the end of the array
};

CommandList cpList = {cpCmds, NULL};

// =============================================================================
//  Construction / destruction
// =============================================================================

commandProcessor::commandProcessor()
    : rb(new ringBuffer(2048))
    , ca(new charAllocate(512))
    , serial(NULL)
    , commands(NULL)
    , echo(false)
    , mute(false)
    , caseSensitive(true)
    , numArgs(0)
    , numStreams(0)
    , error(0)
{
    // Initialise stream arrays to safe defaults.
    for (int i = 0; i < MAX_STREAMS; i++)
    {
        streams[i]      = NULL;
        doNotProcess[i] = false;
    }

    // Wire the built-in bool commands to their instance variables now that
    // `this` is fully constructed.
    cpCmds[0].pointer = (void *)&echo;
    cpCmds[1].pointer = (void *)&mute;
    cpCmds[2].pointer = (void *)&caseSensitive;

    // Register the module-level singleton used by the built-in callbacks.
    CP = this;

    registerCommands(&cpList);
}

commandProcessor::~commandProcessor()
{
    delete rb;
    delete ca;
    // streams[] entries are not owned by this class — do not delete them.
    if (CP == this) CP = NULL;
}

// =============================================================================
//  Stream management
// =============================================================================

void commandProcessor::registerStream(Stream *s)
{
    if (s == NULL) return;
    if (numStreams >= MAX_STREAMS)
    {
        // Array is full — silently ignore.  Increase MAX_STREAMS if needed.
        return;
    }
    streams[numStreams]      = s;
    doNotProcess[numStreams] = false;
    numStreams++;

    // The first registered stream becomes the default output.
    if (numStreams == 1) serial = s;
}

bool commandProcessor::selectStream(Stream *s)
{
    for (int i = 0; i < numStreams; i++)
    {
        if (streams[i] == s)
        {
            serial = s;
            return true;
        }
    }
    return false;
}

Stream *commandProcessor::selectedStream(void)
{
    return serial;
}

/**
 * @brief Enable or disable processing of a registered Stream.
 *
 * Renamed from DoNotprocessStream to setStreamActive for clarity and
 * consistent capitalisation.  The semantics are inverted so that passing
 * `true` means "keep processing this stream" (less surprising for callers).
 */
void commandProcessor::setStreamActive(Stream *s, bool active)
{
    for (int i = 0; i < numStreams; i++)
    {
        if (streams[i] == s)
        {
            doNotProcess[i] = !active;
            return;
        }
    }
}

// =============================================================================
//  Command table management
// =============================================================================

void commandProcessor::registerCommands(CommandList *cmdList)
{
    if (cmdList == NULL) return;

    if (commands == NULL)
    {
        // First registration — becomes the head of the list.
        commands       = cmdList;
        cmdList->next  = NULL;
        return;
    }

    // Walk to the tail and append.
    CommandList *cl = commands;
    while (cl->next != NULL) cl = (CommandList *)cl->next;
    cl->next      = cmdList;
    cmdList->next = NULL;
}

// =============================================================================
//  Processing
// =============================================================================

int commandProcessor::processStreams(void)
{
    for (int i = 0; i < numStreams; i++)
    {
        if (doNotProcess[i]) continue;
        while (streams[i]->available() > 0)
        {
            serial = streams[i]; // route replies back to the stream that sent the command
            char c = (char)serial->read();
            if (echo) serial->write(c);
            rb->put(c);
        }
    }
    return numStreams;
}

// =============================================================================
//  Command dispatch
// =============================================================================

/**
 * @brief Attempt to execute a single matched Command.
 *
 * Handles argument-count validation, then dispatches to the appropriate
 * typed handler.  Returns false (without sending NAK) if the command cannot
 * be handled so that the caller can decide what error to report.
 */
bool commandProcessor::processCommand(Command *c)
{
    // Optional fixed-argument-count check.
    if ((c->nargs > -1) && (c->nargs != numArgs)) return false;

    if (c->type != CMDfunction)
    {
        // For get/set commands, validate the G/S prefix argument count:
        //   G (get): must have 0 arguments after the command token.
        //   S (set): must have exactly 1 argument.
        char prefix = (char)toupper(cmd[0]);
        if (prefix == 'G' && numArgs > 0)  return false;
        if (prefix == 'S' && numArgs != 1) return false;
    }

    if (c->pointer == NULL) return false;

    char   token[MAXTOKEN];
    String tokenS;

    switch (c->type)
    {
        // ------------------------------------------------------------------
        case CMDstr:
            if (numArgs == 0)
            {
                sendACK(false);
                println((char *)c->pointer);
            }
            else // numArgs == 1
            {
                rb->getToken(token, DELIM);
                if (c->options != NULL)
                    if (strstr((char *)c->options, token) == NULL) return false;
                strcpy((char *)c->pointer, token);
                sendACK();
            }
            return true;

        // ------------------------------------------------------------------
        case CMDint:
            if (numArgs == 0)
            {
                sendACK(false);
                println(*(int *)c->pointer);
            }
            else
            {
                rb->getToken(token, DELIM);
                tokenS = token;
                if (c->options != NULL)
                {
                    int lo = ((int *)c->options)[0];
                    int hi = ((int *)c->options)[1];
                    if (tokenS.toInt() < lo || tokenS.toInt() > hi) return false;
                }
                *(int *)c->pointer = (int)tokenS.toInt();
                sendACK();
            }
            return true;

        // ------------------------------------------------------------------
        case CMDfloat:
            if (numArgs == 0)
            {
                sendACK(false);
                println(*(float *)c->pointer);
            }
            else
            {
                rb->getToken(token, DELIM);
                tokenS = token;
                if (c->options != NULL)
                {
                    float lo = ((float *)c->options)[0];
                    float hi = ((float *)c->options)[1];
                    if (tokenS.toFloat() < lo || tokenS.toFloat() > hi) return false;
                }
                *(float *)c->pointer = tokenS.toFloat();
                sendACK();
            }
            return true;

        // ------------------------------------------------------------------
        case CMDdouble:
            // NOTE: ACK is sent AFTER parsing so that a bad argument does not
            // get acknowledged.  This matches the behaviour of all other types.
            if (numArgs == 0)
            {
                sendACK(false);
                println(*(double *)c->pointer);
            }
            else
            {
                rb->getToken(token, DELIM);
                // Use sscanf for double precision; String::toFloat() loses bits.
                double d;
                if (sscanf(token, "%lf", &d) != 1) return false;
                *(double *)c->pointer = d;
                sendACK();
            }
            return true;

        // ------------------------------------------------------------------
        case CMDbool:
            if (numArgs == 0)
            {
                sendACK(false);
                println(*(bool *)c->pointer);
            }
            else
            {
                rb->getToken(token, DELIM);
                tokenS = token;
                if (!caseSensitive) tokenS.toUpperCase();
                if      (tokenS == "TRUE")  *(bool *)c->pointer = true;
                else if (tokenS == "FALSE") *(bool *)c->pointer = false;
                else return false;
                sendACK();
            }
            return true;

        // ------------------------------------------------------------------
        case CMDbyte:
            if (numArgs == 0)
            {
                sendACK(false);
                println(*(uint8_t *)c->pointer);
            }
            else
            {
                rb->getToken(token, DELIM);
                tokenS = token;
                *(uint8_t *)c->pointer = (uint8_t)tokenS.toInt();
                sendACK();
            }
            return true;

        // ------------------------------------------------------------------
        case CMDfunction:
        {
            void (*function)(void) = (void (*)(void))c->pointer;
            function();
            return true;
        }

        // ------------------------------------------------------------------
        default:
            return false;
    }
    // Unreachable, but satisfies some compilers.
    return false;
}

/**
 * @brief Search all registered CommandLists for a token that matches `cmd`.
 *
 * Matching rules:
 *  - If the stored cmd starts with '?', the incoming token may start with
 *    'G' or 'S' (case-insensitive); the rest of both strings are compared
 *    starting at index 1.
 *  - The comparison is case-insensitive when caseSensitive == false.
 *
 * Length check fix: for '?'-prefix entries the stored length includes '?'
 * (e.g. "?ECHO" = 5 chars).  The incoming token (e.g. "GECHO") is also 5
 * chars, so the lengths are equal and the guard works correctly — the loop
 * then starts at j=1 to skip the prefix character on both sides.
 */
Command *commandProcessor::findCommand(void)
{
    if (commands == NULL) return NULL;

    String cmdS = cmd;
    cmdS.trim();
    if (!caseSensitive) cmdS.toUpperCase();

    CommandList *cl = commands;
    while (cl != NULL)
    {
        for (int i = 0; cl->cmds[i].cmd != NULL; i++)
        {
            // Quick length filter before the character-by-character compare.
            if (cmdS.length() != strlen(cl->cmds[i].cmd)) { continue; }

            // Decide where to start the character comparison.
            // For '?' entries the stored cmd[0] is '?' and the incoming
            // token's [0] is 'G' or 'S' — both are skipped by starting j=1.
            int  j     = 0;
            bool match = true;

            if (cl->cmds[i].cmd[0] == '?')
            {
                char p = (char)toupper((unsigned char)cmdS[0]);
                if (p == 'G' || p == 'S')
                    j = 1; // skip the prefix on both strings
                else
                    match = false; // not a valid get/set prefix
            }

            for (; match && (unsigned int)j < cmdS.length(); j++)
            {
                char incoming = cmdS[j];
                char stored   = caseSensitive
                                    ? cl->cmds[i].cmd[j]
                                    : (char)toupper((unsigned char)cl->cmds[i].cmd[j]);
                if (incoming != stored) match = false;
            }

            if (match) return &cl->cmds[i];
        }
        cl = (CommandList *)cl->next;
    }
    return NULL;
}

/**
 * @brief Consume one complete command line from the ring buffer and dispatch it.
 * @return true if a line was consumed (even if the command was invalid);
 *         false if no complete line is available yet.
 */
bool commandProcessor::processCommands(void)
{
    if (rb->lines() <= 0) return false;

    int lines = rb->lines();

    // Read the command token.
    rb->getToken(cmd, DELIM, MAXTOKEN);
    if (strlen(cmd) == 0) return true; // blank line — discard silently

    // Determine how many comma-separated arguments follow on the same line.
    if (lines != rb->lines())
        numArgs = 0;                        // getToken consumed the whole line
    else
        numArgs = rb->tokensInLine(DELIM);  // count remaining tokens

    // Look up and dispatch.
    Command *c = findCommand();
    if (c == NULL)
    {
        sendNAK(ERR_CMD);
    }
    else if (c->type == CMDfunction)
    {
        // Function commands do their own argument validation internally.
        if (!processCommand(c)) sendNAK();
    }
    else
    {
        if (!processCommand(c)) sendNAK(ERR_ARG);
    }

    // Discard any remaining tokens on this line, then reset the scratch allocator.
    if (lines == rb->lines()) rb->clearLine();
    ca->clear();
    return true;
}

// =============================================================================
//  Output helpers
// =============================================================================

/** Print a bare newline (used by all println() wrappers). */
void commandProcessor::print(void)
{
    if (mute || serial == NULL) return;
    serial->println();
}

void commandProcessor::print(char *val)
{
    if (mute || serial == NULL) return;
    serial->print(val);
}

void commandProcessor::print(const char *val)
{
    if (mute || serial == NULL) return;
    serial->print(val);
}

/** Prints the boolean as the human-readable string "TRUE" or "FALSE". */
void commandProcessor::print(bool val)
{
    if (mute || serial == NULL) return;
    serial->print(val ? "TRUE" : "FALSE");
}

void commandProcessor::print(int val, int fmt)
{
    if (mute || serial == NULL) return;
    serial->print(val, fmt);
}

void commandProcessor::print(uint32_t val, int fmt)
{
    if (mute || serial == NULL) return;
    serial->print(val, fmt);
}

void commandProcessor::print(float val, int fmt)
{
    if (mute || serial == NULL) return;
    serial->print(val, fmt);
}

void commandProcessor::print(double val, int fmt)
{
    if (mute || serial == NULL) return;
    serial->print(val, fmt);
}

void commandProcessor::print(uint8_t val, int fmt)
{
    if (mute || serial == NULL) return;
    serial->print(val, fmt);
}

// =============================================================================
//  Protocol output
// =============================================================================

void commandProcessor::sendACK(bool sendNL)
{
    if (mute || serial == NULL) return;
    if (sendNL) serial->println("\x06");
    else        serial->print("\x06");
}

void commandProcessor::sendNAK(int errNum)
{
    error = errNum;
    if (mute || serial == NULL) return;
    serial->println("\x15?");
}

// =============================================================================
//  Interactive input helpers
// =============================================================================

/**
 * @brief Display a prompt, block until a full line arrives, and return it.
 *
 * The returned buffer is managed by `ca` (charAllocate).  The caller must
 * release it with `ca->free(buf)` when done.
 */
char *commandProcessor::userInput(const char *message, void (*function)(void))
{
    rb->clear();
    print(message);

    // Poll until a complete line is in the ring buffer.
    while (rb->lines() == 0)
    {
        processStreams();
        if (function != NULL) function();
    }

    // Allocate exactly enough space for the line plus a NUL terminator.
    int   len = rb->lineLength();
    char *buf = (char *)ca->allocate(len + 1);
    if (buf == NULL)
    {
        rb->clearLine();
        return NULL;
    }

    rb->getLine(buf, len + 1);
    return buf;
}

int commandProcessor::userInputInt(const char *message, void (*function)(void))
{
    char *res = userInput(message, function);
    if (res == NULL) return 0;
    int i = String(res).toInt();
    ca->free(res);
    return i;
}

float commandProcessor::userInputFloat(const char *message, void (*function)(void))
{
    char *res = userInput(message, function);
    if (res == NULL) return 0.0f;
    float f = String(res).toFloat();
    ca->free(res);
    return f;
}

// =============================================================================
//  Typed value extraction from the ring buffer
// =============================================================================

/**
 * @brief Extract one delimiter-separated token from the ring buffer.
 *
 * If @p options is non-NULL the token is checked against it (case-folded
 * when caseSensitive == false).  On failure the allocated buffer is released
 * and *val is set to NULL before returning false.
 *
 * Bug fix: the original code wrote `*val[i]` which is pointer arithmetic on
 * `val` (the char** parameter), not on the char array itself.  Corrected to
 * `(*val)[i]`.
 */
bool commandProcessor::getValue(char **val, const char *options)
{
    int len = rb->tokenLength(DELIM);
    if (len == -1) return false;

    *val = (char *)ca->allocate(len + 1);
    if (*val == NULL) return false;

    rb->getToken(*val, DELIM, len + 1);
    if (rb->peek() == DELIM) rb->get(); // consume trailing delimiter

    if (options != NULL)
    {
        // Case-fold the token in place before the options lookup.
        if (!caseSensitive)
            for (unsigned int i = 0; i < strlen(*val); i++)
                (*val)[i] = (char)toupper((unsigned char)(*val)[i]); // FIXED: was *val[i]

        if (strstr(options, *val) == NULL)
        {
            ca->free(*val);
            *val = NULL;
            return false;
        }
    }
    return true;
}

/**
 * @brief Extract one token and parse it as an int, with optional range check.
 *
 * The "no range check" convention is ll == 0 && ul == 0.
 *
 * Bug fix: the original condition was `(ul==0) && (ul==0)` — comparing ul to
 * itself.  Corrected to `ll == 0 && ul == 0`.
 */
bool commandProcessor::getValue(int *val, int ll, int ul, int fmt)
{
    char *res;
    if (!getValue(&res)) return false;

    int i;
    if (fmt == HEX) sscanf(res, "%x", &i);
    else            sscanf(res, "%d", &i);
    ca->free(res);

    bool noRange = (ll == 0 && ul == 0); // FIXED: was (ul==0)&&(ul==0)
    if (noRange || (i >= ll && i <= ul))
    {
        *val = i;
        return true;
    }
    return false;
}

/**
 * @brief Extract one token and parse it as a uint32_t, with optional range check.
 *
 * Bug fix 1: the original scanned into a local `int i` then cast — on
 * platforms where int is 16-bit this silently truncates 32-bit values.
 * Corrected to use uint32_t directly.
 *
 * Bug fix 2: same `(ul==0)&&(ul==0)` range-bypass condition as getValue(int*).
 */
bool commandProcessor::getValue(uint32_t *val, uint32_t ll, uint32_t ul, int fmt)
{
    char *res;
    if (!getValue(&res)) return false;

    uint32_t u;                              // FIXED: was int i
    if (fmt == HEX) sscanf(res, "%lx", &u); // FIXED: was %x into int
    else            sscanf(res, "%lu", &u);
    ca->free(res);

    bool noRange = (ll == 0 && ul == 0);    // FIXED: was (ul==0)&&(ul==0)
    if (noRange || (u >= ll && u <= ul))
    {
        *val = u;
        return true;
    }
    return false;
}

/**
 * @brief Extract one token and parse it as a float, with optional range check.
 *
 * Bug fix: same `(ul==0)&&(ul==0)` range-bypass condition.
 */
bool commandProcessor::getValue(float *val, float ll, float ul)
{
    char *res;
    if (!getValue(&res)) return false;

    float f = String(res).toFloat();
    ca->free(res);

    bool noRange = (ll == 0.0f && ul == 0.0f); // FIXED: was (ul==0)&&(ul==0)
    if (noRange || (f >= ll && f <= ul))
    {
        *val = f;
        return true;
    }
    return false;
}

// =============================================================================
//  Accessors
// =============================================================================

char *commandProcessor::getCMD(void)
{
    return cmd;
}

int commandProcessor::getNumArgs(void)
{
    return numArgs;
}

bool commandProcessor::checkExpectedArgs(int num)
{
    if (numArgs != num)
    {
        sendNAK(ERR_ARG);
        return false;
    }
    return true;
}
