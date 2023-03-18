package org.frc1410.framework.util.log.transport;

import org.frc1410.framework.util.log.Log;
import org.jetbrains.annotations.NotNull;

@FunctionalInterface
public interface Transport {

	void accept(@NotNull Log log);
}