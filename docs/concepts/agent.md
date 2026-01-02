# Agent

## Motivation

Traditional robot programming requires manually coding every behavior. By contrast, LLM-powered agents (in conjunction with [skills](./skills.md)) allow you to instruct robots at a higher level of abstraction, in natural language. Tell it to "go to the kitchen" and it figures out which skills to call and when.

> [!TIP]
> New to agents? Start with the [Equip an agent with skills](../tutorials/skill_with_agent/tutorial.md) tutorial for a hands-on introduction.

```python
from dimos.agents2.agent import llm_agent
from dimos.agents2.cli.human import human_input
from dimos.agents2.skills.navigation import navigation_skill
from dimos.core.blueprints import autoconnect
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic

# Create an agentic robot system
blueprint = autoconnect(
    basic,                   # Hardware, navigation, mapping
    navigation_skill(),      # Exposes navigation as agent-callable skills
    llm_agent(               # The reasoning agent
        system_prompt="You are a helpful robot assistant."
    ),
    human_input()           # CLI for sending commands to the agent
)
```

<!-- Evidence: dimos/robot/unitree_webrtc/unitree_go2_blueprints.py:110-116 - Real "agentic" blueprint showing this pattern -->

## Situating Agents vis-a-vis other DimOS concepts

Agents are [Modules](./modules.md), so they inherit streams, RPC, lifecycle management, and distributed deployment.

However, an agent doesn't see information from streams directly. If you want to feed information to an agent, you need to do it via [skills](./skills.md).

## How to build agentic systems

Check out these tutorials for a better answer:

* [Equip an agent with a skill](../tutorials/skill_with_agent/tutorial.md)
* [Build a multi-agent RoboButler system](../tutorials/multi_agent/tutorial.md)
<!-- TODO: Add multiagent tutorial -->

But the short answer is, add an agent to the blueprint using `llm_agent()`.

```python
from dimos.agents2.agent import llm_agent

agent_bp = llm_agent(
    system_prompt="You are a warehouse robot. Focus on navigation and inventory tasks.",
    model="gpt-4o-mini",
    provider="openai"
)
```

DimOS supports multiple LLM providers through Langchain - switching requires only configuration changes.

<!-- Evidence: dimos/agents2/agent.py:375 - llm_agent = LlmAgent.blueprint factory function -->
<!-- Evidence: dimos/agents2/spec.py:130-143 - AgentConfig dataclass with system_prompt, model, provider, model_instance fields -->
<!-- Evidence: dimos/agents2/agent.py:195-197 - init_chat_model usage with provider and model params -->
<!-- Evidence: dimos/agents2/spec.py:45-47 - Provider enum dynamically created from langchain's 20 supported providers -->

## Skill discovery (also discussed in tutorials)

For an agent to discover skills, the skill module must register itself. The simplest way is to subclass `SkillModule`:

```python
from dimos.core.skill_module import SkillModule
from dimos.protocol.skill.skill import skill

class NavigationSkills(SkillModule):
    """Module providing navigation capabilities."""

    @skill()
    def navigate_to(self, location: str) -> str:
        """Navigate to the specified location."""
        # Implementation...
        return f"Navigating to {location}"
```

<!-- Evidence: dimos/core/skill_module.py:20-26 - SkillModule adds set_LlmAgent_register_skills -->

`SkillModule` is just `Module` with a `set_LlmAgent_register_skills` hook that registers its skills when composed with an agent. The naming convention `set_<ModuleName>_<method>` tells the system to call this method with the matching module's method when the blueprint is built.

<!-- Evidence: dimos/core/blueprints.py:396-405 - Convention: methods starting with set_ are called with matching RPC references -->

When skills are registered, the agent:

1. Converts `@skill` methods to LLM tool definitions (using the docstring as the tool description -- this is why it's important to have good docstrings for those methods)
2. Exposes them to the LLM for reasoning

<!-- Evidence: dimos/agents2/agent.py:350-351 - get_tools() retrieves tools from coordinator -->
<!-- Evidence: dimos/protocol/skill/schema.py:63-103 - function_to_schema extracts docstring -->

## The agent loop

The agent runs an event-driven reasoning loop:

1. **Invoke LLM** - With conversation history and skill state
2. **Execute tool calls** - Dispatch requested skills to coordinator
3. **Wait for updates** - Suspend until skills produce results
4. **Process results** - Transform skill outputs into messages
5. **Repeat** - Continue until the skills that were initialized with `Return.call_agent` or `Stream.call_agent` -- what we might call *active* skills for short -- have completed.

<!-- Evidence: dimos/agents2/agent.py - agent_loop() implementation -->

The loop handles *long-running operations* without blocking. Navigation takes 30 seconds? The agent waits, then resumes reasoning with results.

<!-- Evidence: dimos/agents2/agent.py:304 - await coordinator.wait_for_updates() enables async waiting -->

## How agents receive information

> [!IMPORTANT]
> If you want to get information to an agent, you need to do that with skills.

There are two broad ways to get information to an agent. You can either (i) give it skills that it can use to get information -- think of a coding agent and its search tool(s) -- or (ii) stream updates from skills. For more details, see [the Skills concept guide](./skills.md).

## State management

Agents follow a one-way lifecycle - once stopped, they stay stopped:

```ascii
INITIALIZED → STARTED → RUNNING → STOPPED (terminal)
```

Stopped agents **cannot restart**. This prevents mixing old and new conversation contexts. To resume operations, create a fresh agent instance.

<!-- Evidence: dimos/agents2/agent.py:209-212 - stop() sets _agent_stopped = True -->
<!-- Evidence: dimos/agents2/agent.py:204-206 - start() does not reset _agent_stopped -->
<!-- Evidence: dimos/agents2/agent.py:250-256 - agent_loop() checks flag and returns early -->

This one-way pattern supports explicit state management - each agent instance represents a single conversation session with its own history and context.

## Common use cases

* **Exploration and mapping** - Agent plans exploration pattern, navigates to waypoints, tags rooms in memory, reports findings.

* **Object search and navigation** - Agent searches memory for target object, explores to locate it if not found, navigates to object's location, confirms arrival.

* **Guided tours and explanations** - Agent navigates to key locations, describes what's at each, answers questions about equipment and procedures.

## See also

### Hands-on tutorials

* [Equip an agent with skills](../tutorials/skill_with_agent/tutorial.md)

* [Build a multi-agent RobotButler](../tutorials/multi_agent/tutorial.md)

### Related concepts

* [Skills](./skills.md) - Methods that agents can discover and invoke
* [Blueprints](./blueprints.md) - Composing agents, skills, and hardware into systems
* [Modules](./modules.md) - The foundational abstraction that agents build upon

### API reference

* [Agents API](../api/agents.md) - API reference for agent classes, message types, and configuration
