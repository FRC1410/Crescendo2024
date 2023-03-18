package org.frc1410.framework.util.log.transport;

import org.frc1410.framework.util.log.Log;
import org.jetbrains.annotations.NotNull;

class ConsoleTransport implements Transport {

	@Override
	public void accept(@NotNull Log log) {
		System.out.printf("%s[%-24s] %5s: %s\u001b[0m\n", log.level.getAnsiColor(), log.source.getName(), log.level, log.message);
	}
}
