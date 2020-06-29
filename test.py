from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
import asyncio

async def my_coroutine():
    session = PromptSession()
    while True:
        with patch_stdout():
            result = await session.prompt_async('Say something: ')
        print('You said: %s' % result)

if __name__ == "__main__":
	# Start the main function
	asyncio.ensure_future(my_coroutine())

	# Runs the event loop until the program is canceled with e.g. CTRL-C
	asyncio.get_event_loop().run_forever()