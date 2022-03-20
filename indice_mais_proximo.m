function [indiceMaisProximo] = indice_mais_proximo(listaDeValores, valorAProcurarIndiceMaisProximo)


[~, indiceMaisProximo] = min(abs(listaDeValores - valorAProcurarIndiceMaisProximo));

end

