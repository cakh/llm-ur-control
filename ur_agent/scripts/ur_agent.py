#!/usr/bin/env python3
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
#  Modifications made by the Chair of Handling and Assembly Technology, TU Chemnitz, 2024:
#  - Updated the example for use with ROS2 and UR5e robots.
#  - Integrated with the LLM framework in this package.
#  - Changes the code to work with ROS2


import asyncio
import os
import dotenv
import pyinputplus as pyip
import rclpy
from langchain.agents import tool, Tool
from rich.console import Console
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from rosa import ROSA
import tools.ur as ur_tools
from help import get_help
from llm import get_llm
from prompts import get_prompts

# Typical method for defining tools in ROSA
@tool
def cool_ur_tool():
    """A cool ur tool that doesn't really do anything."""
    return "This is a cool ur tool! It doesn't do anything, but it's cool."

def move_home(self, input: str):
    return f"""
    Ok, we're moving to home position!

    <ROSA_INSTRUCTIONS>
        You should now use your tools to make all the joints to position 0 degrees.
    </ROSA_INSTRUCTIONS>
    """

class URAgent(ROSA):
    def __init__(self, streaming: bool = True, verbose: bool = True):
        self.__blacklist = []
        self.__prompts = get_prompts()
        self.__llm = get_llm(streaming=streaming)

        # Another method for adding tools
        move_home_tool = Tool(
            name="move_home",
            func=move_home,
            description="Make the ur move to its home position",
        )

        super().__init__(
            ros_version=2,  # Specify ROS version
            llm=self.__llm,
            tools=[cool_ur_tool, move_home_tool],  # Add tools specific to your application
            tool_packages=[ur_tools],  # Add tool packages specific to your application
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=False,
            streaming=streaming,
        )
        print("Tools registered:")

        self.examples = [
            "Give me a ROS tutorial using the ur robot.",
            "Show me how to move the ur.",
            "Draw a 5-point star using the ur tcp.",
            "Give me a list of nodes, topics, services, params, and log files.",
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": lambda: self.clear(),
        }


    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the ROSA-UR5e agent 🤖. How can I help you today?\n"
        )
        greeting.stylize("frame bold blue")
        greeting.append(
            f"Try {', '.join(self.command_handler.keys())} or exit.",
            style="italic",
        )
        return greeting

    def choose_example(self):
        """Get user selection from the list of examples."""
        return pyip.inputMenu(
            self.examples,
            prompt="\nEnter your choice and press enter: \n",
            numbered=True,
            blank=False,
            timeout=60,
            default="1",
        )

    async def clear(self):
        """Clear the chat history."""
        self.clear_chat()
        self.last_events = []
        self.command_handler.pop("info", None)
        os.system("clear")

    def get_input(self, prompt: str):
        """Get user input from the console."""
        return pyip.inputStr(prompt, default="help")

    async def run(self):
        """
        Run the urAgent's main interaction loop.

        This method initializes the console interface and enters a continuous loop to handle user input.
        It processes various commands including 'help', 'examples', 'clear', and 'exit', as well as
        custom user queries. The method uses asynchronous operations to stream responses and maintain
        a responsive interface.

        The loop continues until the user inputs 'exit'.

        Returns:
            None

        Raises:
            Any exceptions that might occur during the execution of user commands or streaming responses.
        """
        await self.clear()
        console = Console()

        while True:
            console.print(self.greeting)
            input = self.get_input("> ")

            # Handle special commands
            if input == "exit":
                break
            elif input in self.command_handler:
                await self.command_handler[input]()
            else:
                await self.submit(input)

    async def submit(self, query: str):
            self.print_response(query)

    def print_response(self, query: str):
        """
        Submit the query to the agent and print the response to the console.

        Args:
            query (str): The input query to process.

        Returns:
            None
        """
        response = self.invoke(query)
        console = Console()
        content_panel = None

        with Live(
            console=console, auto_refresh=True, vertical_overflow="visible"
        ) as live:
            content_panel = Panel(
                Markdown(response), title="Final Response", border_style="green"
            )
            live.update(content_panel, refresh=True)



def main(args=None):
    ur_agent = URAgent(verbose=True, streaming=True)
    dotenv.load_dotenv(dotenv.find_dotenv())
    try:
        # Use asyncio to run the node's logic
        asyncio.run(ur_agent.run())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()   

    asyncio.run(ur_agent.run())


if __name__ == "__main__":
    main()
