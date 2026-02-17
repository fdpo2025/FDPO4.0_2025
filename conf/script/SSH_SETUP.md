# Configuração SSH para Automação de Robôs

## Opção 1: Chaves SSH (RECOMENDADO) 🔐

### Por que usar chaves SSH?
- ✅ Mais seguro (sem senhas em texto plano)
- ✅ Mais rápido (sem prompts de senha)
- ✅ Mais conveniente (automação completa)
- ✅ Padrão da indústria

### Como configurar:

#### 1. Gerar chave SSH (se ainda não tiver):
```bash
ssh-keygen -t rsa -b 4096 -C "seu_email@exemplo.com"
# Pressione Enter para aceitar localização padrão (~/.ssh/id_rsa)
# Pressione Enter para senha vazia (ou defina uma senha forte)
```

#### 2. Copiar chave pública para os robôs:
```bash
# Para o primeiro robô
ssh-copy-id usuario@robot1.local
# ou
ssh-copy-id usuario@192.168.1.10

# Para o segundo robô
ssh-copy-id usuario@robot2.local
# ou
ssh-copy-id usuario@192.168.1.11
```

#### 3. Testar conexão sem senha:
```bash
ssh usuario@robot1.local  # Não deve pedir senha
ssh usuario@robot2.local  # Não deve pedir senha
```

#### 4. Usar o script normalmente:
```bash
./conf/script/launch_dual_robots.sh \
  --robot1 robot1.local \
  --robot2 robot2.local \
  --path "0 1 2 3"
```

---

## Opção 2: Usar senhas via sshpass (TEMPORÁRIO) ⚠️

### Quando usar:
- Apenas para testes rápidos
- Quando não é possível configurar chaves SSH
- **NÃO recomendado para produção**

### Como usar:

#### 1. Instalar sshpass:
```bash
sudo apt-get install sshpass
```

#### 2. Usar o script com senhas:
```bash
./conf/script/launch_dual_robots.sh \
  --robot1 robot1.local \
  --robot2 robot2.local \
  --pass1 "senha_robot1" \
  --pass2 "senha_robot2" \
  --path "0 1 2 3"
```

### ⚠️ Avisos de Segurança:
- **Senhas aparecem no histórico do bash** (`history`)
- **Senhas aparecem em processos** (`ps aux | grep sshpass`)
- **Senhas podem aparecer em logs**
- **Use apenas em ambientes de desenvolvimento/teste**

### Alternativa mais segura (usar variável de ambiente):
```bash
# Definir senhas como variáveis de ambiente
export ROBOT1_PASS="senha1"
export ROBOT2_PASS="senha2"

# Modificar o script para ler de variáveis de ambiente
# (não implementado por padrão por segurança)
```

---

## Troubleshooting

### Problema: "Permission denied (publickey)"
**Solução**: Configure chaves SSH (Opção 1)

### Problema: "sshpass: command not found"
**Solução**: `sudo apt-get install sshpass`

### Problema: "Host key verification failed"
**Solução**: 
```bash
ssh-keyscan robot1.local >> ~/.ssh/known_hosts
ssh-keyscan robot2.local >> ~/.ssh/known_hosts
```

### Problema: "Connection refused"
**Solução**: 
- Verifique se SSH está rodando nos robôs: `sudo systemctl status ssh`
- Verifique firewall: `sudo ufw status`
- Verifique conectividade de rede: `ping robot1.local`

---

## Melhores Práticas

1. ✅ **Sempre use chaves SSH** em produção
2. ✅ **Use senhas fortes** se precisar usar sshpass temporariamente
3. ✅ **Não commite senhas** no git
4. ✅ **Use diferentes usuários** para diferentes níveis de acesso
5. ✅ **Desative login por senha** nos robôs após configurar chaves:
   ```bash
   # No robô, editar /etc/ssh/sshd_config:
   PasswordAuthentication no
   PubkeyAuthentication yes
   
   # Reiniciar SSH:
   sudo systemctl restart sshd
   ```






